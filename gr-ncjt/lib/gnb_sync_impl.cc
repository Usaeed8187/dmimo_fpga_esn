/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "gnb_sync_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <volk/volk.h>
#include "utils.h"

namespace gr::ncjt
{

static const pmt::pmt_t TIME_KEY = pmt::string_to_symbol("rx_time");
static const pmt::pmt_t RATE_KEY = pmt::string_to_symbol("rx_rate");
static const pmt::pmt_t FREQ_KEY = pmt::string_to_symbol("rx_freq");

gnb_sync::sptr
gnb_sync::make(int nchans, int preamblelen, int dataframelen,
               double samplerate, int pktspersec,
               double acorr_thrd, double xcorr_thrd,
               int max_corr_len, bool debug)
{
    return gnuradio::make_block_sptr<gnb_sync_impl>(
        nchans, preamblelen, dataframelen, samplerate, pktspersec, acorr_thrd,
        xcorr_thrd, max_corr_len, debug);
}

gnb_sync_impl::gnb_sync_impl(int nchans, int preamblelen, int dataframelen,
                             double samplerate, int pktspersec,
                             double acorr_thrd, double xcorr_thrd,
                             int max_corr_len, bool debug)
    : gr::block("gnb_sync",
                gr::io_signature::make(nchans, nchans, sizeof(gr_complex)),
                gr::io_signature::make(nchans, 2 * nchans, sizeof(gr_complex))),
      d_sampling_freq(samplerate), d_wait_interval0(0),  d_acorr_thrd(acorr_thrd), d_xcorr_thrd(xcorr_thrd),
      d_max_corr_len(max_corr_len), d_frame_start(0), // d_prev_frame_count(0), d_prev_frame_time(0.0),
      d_xcorr_fir(gr::filter::kernel::fir_filter_ccc(LTF_SEQ)), d_debug(debug)
{
    if (nchans < 1 || nchans > MAX_CHANS)
        throw std::runtime_error("Currently support 1 to 8 IQ channels");
    d_num_chans = nchans;

    if (samplerate < 10000)
        throw std::runtime_error("invalid sampling frequency specified");
    if (pktspersec <= 0 || pktspersec > 100)
        throw std::runtime_error("invalid sampling packet transmission interval specified");

    if (preamblelen < 0 || preamblelen >= SYM_LEN * MAX_PREAMBLE_SYMS)
        throw std::runtime_error("invalid HT preamble length specified");
    if (preamblelen < 0 || preamblelen > int(samplerate / pktspersec))
        throw std::runtime_error("invalid data frame length specified");

    // total length of HT-LTF and data symbols, excluding HT-SIG and HT-LTF
    d_frame_len = dataframelen + preamblelen - 3 * SYM_LEN; // 3*SYM_LEN;

    d_pkt_interval = 1.0 / (double) pktspersec;

    // assuming legacy preamble of 5 symbol length (L-STF, L-LTF, and L-SIG) and 3 symbols of HT fields
    // gNB start from t0 + T/4 after rx_time adjustment, receive P2 and P3 transmission
    d_wait_interval1 = (int) floor(samplerate * d_pkt_interval / 2) - d_frame_len - 8 * SYM_LEN;
    d_wait_interval2 = (int) floor(samplerate * d_pkt_interval / 2) - d_frame_len - 8 * SYM_LEN;

    d_corr_buf_pos = 0;
    d_corr_buf = malloc_complex(CORR_BUF_LEN);
    d_pwrest_buf = malloc_float(CORR_BUF_LEN);

    d_input_buf = malloc_complex(XCORR_DATA_LEN * nchans);
    d_xcorr_buf = malloc_complex(MAX_XCORR_LEN * nchans);
    d_xcorr_val = malloc_float(1024);

    d_state = RXTIME; // SEARCH;
    d_wait_count = 0;
    d_data_samples = 0;

    d_rx_ready_cnt1 = 0;
    d_rx_ready_cnt2 = 0;
    d_fine_foe_comp2 = 0.0;
    d_fine_foe_comp2 = 0.0;
    d_current_foe_comp1 = 0.0;
    d_current_foe_comp2 = 0.0;
    d_fine_foe_comp1 = 0.0;
    d_fine_foe_comp2 = 0.0;
    d_fine_foe_cnt1 = 0;
    d_fine_foe_cnt2 = 0;
    d_skip_p2_frame = false;
    d_skip_p3_frame = false;

    message_port_register_out(pmt::mp("uhdcmd"));
    message_port_register_out(pmt::mp("rxtime"));
    message_port_register_out(pmt::mp("rxrdy"));

    set_tag_propagation_policy(block::TPP_DONT);

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());
}

gnb_sync_impl::~gnb_sync_impl()
{
    volk_free(d_corr_buf);
    volk_free(d_pwrest_buf);
    volk_free(d_input_buf);
    volk_free(d_xcorr_buf);
    volk_free(d_xcorr_val);
}

void
gnb_sync_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    switch (d_state)
    {
        case SEARCH1:
        case SEARCH2:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * LTF_LEN;
            break;
        }
        case FINESYNC1:
        case FINESYNC2:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 4 * LTF_LEN;
            break;
        }
        default: // DEFRAME
        {
            int nitems = std::max(noutput_items, 8 * LTF_LEN);
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = nitems;
            break;
        }
    } // d_state
}

void
gnb_sync_impl::send_rxstate(bool ready)
{
    dout << "Receiver synchronization " << (ready ? "acquired" : "lost") << std::endl;
    message_port_pub(pmt::mp("rxrdy"), pmt::from_bool(ready));
}

int
gnb_sync_impl::general_work(int noutput_items, gr_vector_int &ninput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_num_chans; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int noutput_samples = 0;

    switch (d_state)
    {
        case RXTIME:
        {
            // send tag command and check current timestamp
            send_tagcmd();

            // skip first 5 seconds for gNB receiver
            if (nitems_read(0) < uint64_t(5 * d_sampling_freq))
            {
                consume_each(min_input_items);
                break;
            }

            // Adjust frame start according to rx_time
            // 1) assuming gNB transmission is aligned with PPS (integer rx_time)
            // 2) TxUE transmission starts with timing offset of 1/4 pkt_interval
            // 3) RxUE transmission starts with timing offset of 3/4 pkt_interval
            double time_fracs = check_rxtime(min_input_items);
            if (time_fracs > 0)
            {
                double intpart;
                double pkt_time_fracs = std::modf(time_fracs / d_pkt_interval, &intpart);
                double pkt_start_adjust = (1.25 - pkt_time_fracs) * d_pkt_interval;
                d_wait_interval0 = int(d_sampling_freq * pkt_start_adjust); // + SYM_LEN * 3;
                d_wait_count = 0;
                d_state = WAIT0;
                uint64_t pkt_start = nitems_read(0) + d_wait_interval0;
                dout << "Adjusting gNB timing: " << pkt_start << std::endl;
                break;
            }

            // no rx_time information available, remove all received samples
            consume_each(min_input_items);
            break;
        }
        case WAIT0:
        {
            d_wait_count += min_input_items;
            int new_data_count = d_wait_count - d_wait_interval0;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                d_data_samples = 0;
                d_state = FINESYNC1; //SEARCH1;
                std::cout << "Phase 2: entering SEARCH" << std::endl;
            }
            consume_each(min_input_items);
            break;
        }
        case SEARCH1:
        {
            d_fine_foe_cnt1 = 0;
            d_fine_foe_comp1 = 0.0;
            d_current_foe_comp1 = 0.0;

            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp1);
            if (start_pos > 0)
            {
                d_frame_start = nitems_read(0) + start_pos;
                dout << "Phase 2: Packet start at " << d_frame_start << std::endl;
                consume_each(start_pos);
            } /* else {
                // frame start not found, discard samples
                // consume_each(buffer_len - 2 * STF_LEN);
                dout << "Phase 2: Packet start not found" << std::endl;
                break;
            }*/
            d_state = FINESYNC1;
            break;
        }
        case FINESYNC1:
        {
            check_rxtime(min_input_items);

            int htsig_start = fine_sync(input_items, 3 * LTF_LEN,
                                        d_current_foe_comp1, d_fine_foe_comp1,
                                        d_rx_ready_cnt1, d_fine_foe_cnt1);
            if (htsig_start < 0)
            {
                d_skip_p2_frame = true;
                htsig_start = SYM_LEN * 7; // use default value
                std::cout << "Phase 2: Fine synchronize failed for Tx UE!" << std::endl;
            }

            d_frame_start = nitems_read(0) + htsig_start;
            dout << "Phase 2: Frame start at " << d_frame_start << std::endl;
            d_data_samples = 0;
            d_state = P2DEFRAME;
            send_tagcmd();

            consume_each(htsig_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            if (d_rx_ready_cnt1 >= 9 && d_rx_ready_cnt2 >= 9) // changing to ready state
                send_rxstate(true);
            if (d_rx_ready_cnt1 < 100)
                d_rx_ready_cnt1 += 1;

            break;
        }
        case P2DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len)
            {
                noutput_samples = d_frame_len - d_data_samples;
                d_wait_count = 0;
                d_state = WAIT1;
            }

            if (d_skip_p2_frame)
            {
                d_data_samples += noutput_samples;
                consume_each(noutput_samples);
                noutput_samples = 0;
                break;
            }

            // add packet frame start tag
            if (d_data_samples == 0)
            {
                for (int ch = 0; ch < d_num_chans; ch++)
                    add_item_tag(ch,
                                 nitems_written(ch),
                                 pmt::string_to_symbol("frame_start"),
                                 pmt::from_long(d_frame_len),
                                 pmt::string_to_symbol(name()));
            }

            for (int i = 0; i < noutput_samples; i++)
            {
                gr_complex comp_val = exp(gr_complex(0, d_current_foe_comp1 * (double) d_data_samples));
                for (int ch = 0; ch < d_num_chans; ch++)
                {
                    auto in = (const gr_complex *) input_items[ch];
                    auto out = (gr_complex *) output_items[ch];
                    out[i] = in[i] * comp_val;
                }
                d_data_samples += 1;
            }

            check_rxtime(noutput_samples);
            consume_each(noutput_samples);
            for (int ch = 0; ch < d_num_chans; ch++)
            {
                produce(ch, noutput_samples);
                produce(d_num_chans + ch, 0);
            }

            break;
        }
        case WAIT1:
        {
            d_wait_count += min_input_items;
            int new_data_count = d_wait_count - d_wait_interval1;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                d_data_samples = 0;
                d_state = FINESYNC2; // SEARCH2;
            }
            if (d_wait_count < 2048)
                check_rxtime(noutput_samples);
            consume_each(min_input_items);
            break;
        }
        case SEARCH2:
        {
            d_fine_foe_cnt2 = 0;
            d_fine_foe_comp2 = 0.0;
            d_current_foe_comp2 = 0.0;

            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp2);
            if (start_pos < 0)
            {
                // frame start not found, discard samples
                consume_each(buffer_len - 2 * STF_LEN);
                break;
            }
            d_frame_start = nitems_read(0) + start_pos;
            dout << "Phase 3: Packet start at " << d_frame_start << std::endl;
            consume_each(start_pos);
            for (int ch = 0; ch < 2 * d_num_chans; ch++)
                produce(ch, 0);
            d_state = FINESYNC2;
            break;
        }
        case FINESYNC2:
        {
            check_rxtime(min_input_items);

            int htsig_start = fine_sync(input_items, 3 * LTF_LEN,
                                        d_current_foe_comp2, d_fine_foe_comp2,
                                        d_rx_ready_cnt2, d_fine_foe_cnt2);
            if (htsig_start < 0)
            {
                d_skip_p3_frame = true;
                htsig_start = 188; // use default value
                std::cout << "Phase 3 :Fine synchronize failed for RxUE!" << std::endl;
            }

            d_frame_start = nitems_read(0) + htsig_start;
            dout << "Phase 3: Frame start at " << d_frame_start << std::endl;
            d_data_samples = 0;
            d_state = P3DEFRAME;
            send_tagcmd();

            consume_each(htsig_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            if (d_rx_ready_cnt1 >= 9 && d_rx_ready_cnt2 >= 9) // changing to ready state
                send_rxstate(true);
            if (d_rx_ready_cnt1 < 100)
                d_rx_ready_cnt1 += 1;

            break;
        }
        case P3DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len)
            {
                noutput_samples = d_frame_len - d_data_samples;
                d_state = WAIT2;
            }

            if (d_skip_p3_frame)
            {
                d_data_samples += noutput_samples;
                consume_each(noutput_samples);
                noutput_samples = 0;
                break;
            }

            // add packet frame start tag
            if (d_data_samples == 0)
            {
                for (int ch = d_num_chans; ch < 2 * d_num_chans; ch++)
                    add_item_tag(ch,
                                 nitems_written(ch),
                                 pmt::string_to_symbol("frame_start"),
                                 pmt::from_long(d_frame_len),
                                 pmt::string_to_symbol(name()));
            }

            for (int i = 0; i < noutput_samples; i++)
            {
                gr_complex comp_val = exp(gr_complex(0, d_current_foe_comp2 * (double) d_data_samples));
                for (int ch = 0; ch < d_num_chans; ch++)
                {
                    auto in = (const gr_complex *) input_items[ch];
                    auto out = (gr_complex *) output_items[d_num_chans + ch];
                    out[i] = in[i] * comp_val;
                }
                d_data_samples += 1;
            }

            check_rxtime(noutput_samples);
            consume_each(noutput_samples);
            for (int ch = 0; ch < d_num_chans; ch++)
            {
                produce(ch, 0);
                produce(d_num_chans + ch, noutput_samples);
            }

            break;
        }
        case WAIT2:
        {
            d_wait_count += min_input_items;
            int new_data_count = d_wait_count - d_wait_interval2;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                std::cout << "Returning to Sync Search" << std::endl;
                d_state = FINESYNC1; // SEARCH1;
            }
            if (d_wait_count < 2048)
                check_rxtime(noutput_samples);
            consume_each(min_input_items);
            for (int ch = 0; ch < 2 * d_num_chans; ch++)
                produce(ch, 0);
            break;
        }
    }

    return WORK_CALLED_PRODUCE;
}


void
gnb_sync_impl::send_tagcmd()
{
    pmt::pmt_t cmdmsg = pmt::make_dict();
    cmdmsg = pmt::dict_add(cmdmsg, pmt::mp("tag"), pmt::mp(""));
    message_port_pub(pmt::mp("uhdcmd"), cmdmsg);
}

void
gnb_sync_impl::send_rxstate_message(bool ready)
{
    pmt::pmt_t dict = pmt::make_dict();
    dict = pmt::dict_add(dict, pmt::mp("rxrdy"), _id);
    message_port_pub(pmt::mp("rxrdy"), dict);
}

// This function calculate the correct absolute time for the current frame start position
// and send this to tx_framing block
double
gnb_sync_impl::check_rxtime(int rx_windows_size)
{
    // Search for rx_time tag
    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, rx_windows_size, TIME_KEY);
    if (!d_tags.empty())
    {
        // current receiver position of (first sample of the IQ  buffer)
        uint64_t current_pos = nitems_read(0);
        // extract integer and fractional timestamp
        auto pt = pmt::to_tuple(d_tags[0].value);
        uint64_t time_secs = pmt::to_uint64(pmt::tuple_ref(pt, 0));
        double time_fracs = pmt::to_double(pmt::tuple_ref(pt, 1));
        // std::cout << "Current time " << time_secs << ":" << time_fracs << " (" << current_pos << ")" << std::endl;

        // update hardware absolute time corresponding to the frame start
        // d_prev_frame_count = current_pos;
        // d_prev_frame_time = time_fracs; // (double) time_secs + time_fracs;

        if ((d_state == P2DEFRAME || d_state == P3DEFRAME || d_state == WAIT1 || d_state == WAIT2)
            && current_pos >= d_frame_start && d_frame_start > 0)
            // && current_pos < d_frame_start + uint64_t(d_wait_interval))
        {
            // get the actual time for current frmstart
            double frmstart_time = time_fracs + double(time_secs);
            frmstart_time -= double(current_pos - d_frame_start) / d_sampling_freq;
            double intpart;
            double timefracs = std::modf(frmstart_time, &intpart);
            uint64_t timesecs = uint64_t(intpart);
            // std::cout << "Packet start at " << timesecs << ":" << timefracs << " " << (current_pos - d_frame_start) << std::endl;
            // send frame start time via message (integer and fractional parts of the frmstart time)
            auto rxtime_msg = pmt::make_tuple(pmt::from_uint64(timesecs), pmt::from_double(timefracs));
            message_port_pub(pmt::mp("rxtime"), rxtime_msg);
        }
        return time_fracs;
    }
    return 0;
}

/*
 * search for frame start using L-LTS auto-corr peaks
 */
int
gnb_sync_impl::sync_search(const gr_vector_const_void_star &input_items, int buffer_len,
                           float &current_foe_comp)
{
    float power_est = 0;  // current auto-correlation sum
    gr_complex auto_corr = 0;
    int peak_start = -1, peak_duration = 0;
    int frame_start_pos = -1;

    // initial segment
    for (int i = 0; i < (CORR_WINDOW - 1); i++)
    {
        float sig_pwr = 0;
        gr_complex corr_val = 0;
        int delay_idx = i + CORR_DELAY;
        for (int ch = 0; ch < d_num_chans; ch++)
        {
            const auto *in = (const gr_complex *) input_items[ch];
            corr_val += conj(in[i]) * in[delay_idx];
            sig_pwr += norm(in[delay_idx]);
        }
        auto_corr += corr_val;
        d_corr_buf[d_corr_buf_pos] = corr_val;
        power_est += sig_pwr;
        d_pwrest_buf[d_corr_buf_pos] = sig_pwr;

        d_corr_buf_pos = (d_corr_buf_pos + 1) & (CORR_BUF_LEN - 1);  // modulo operation
    }

    // continuous segment
    for (int i = CORR_WINDOW - 1; i < buffer_len - CORR_DELAY; i++)
    {
        // new sample
        int delay_idx = i + CORR_DELAY;
        gr_complex corr_val = 0;
        float sig_pwr = 0;
        for (int ch = 0; ch < d_num_chans; ch++)
        {
            auto in = (const gr_complex *) input_items[ch];
            corr_val += conj(in[i]) * in[delay_idx];
            sig_pwr += norm(in[delay_idx]);
        }
        auto_corr += corr_val;
        d_corr_buf[d_corr_buf_pos] = corr_val;
        power_est += sig_pwr;
        d_pwrest_buf[d_corr_buf_pos] = sig_pwr;

        // check auto-corr peaks, use noise lower-bound 1e-2 to avoid false detection
        float corr_norm = norm(auto_corr) / (power_est * power_est + 1e-8f);
        if (corr_norm >= d_acorr_thrd)
        {
            if (peak_duration == 0)
                peak_start = i;
            peak_duration++;
        }
        else
        {
            peak_start = -1;
            peak_duration = 0;
        }

        // remove old sample
        d_corr_buf_pos = (d_corr_buf_pos + 1) & (CORR_BUF_LEN - 1);
        int oldest_corr_pos = (d_corr_buf_pos + (CORR_BUF_LEN - CORR_WINDOW)) & (CORR_BUF_LEN - 1);
        auto_corr -= d_corr_buf[oldest_corr_pos];
        power_est -= d_pwrest_buf[oldest_corr_pos];

        // break when S-LTF is found
        if (peak_duration >= 3 * CORR_DELAY)
        {
            // frame start should be within CORR_DELAY samples before the start of L-STF
            frame_start_pos = peak_start - 3 * CORR_DELAY;
            dout << "Autocorr peak found at " << peak_start << " (peak duration " << peak_duration << ")" << std::endl;
            break;
        }
    }

    if (frame_start_pos >= 0)
    {
        gr_complex corr_foe = 0;
        // avoid the initial CORR_DELAY samples, 9*CORR_DELAY samples are usable
        for (int k = frame_start_pos + 2 * CORR_DELAY; k < frame_start_pos + 9 * CORR_DELAY; k++)
        {
            int delay_idx = k + CORR_DELAY;
            // sum over all channels, assuming same LO frequency across all receivers
            for (int ch = 0; ch < d_num_chans; ch++)
            {
                auto in = (const gr_complex *) input_items[ch];
                corr_foe += in[k] * conj(in[delay_idx]);
            }
        }
        current_foe_comp = arg(corr_foe) / (float) CORR_DELAY; // FOE compensation in radians
        float foe_comp_hz = current_foe_comp * d_sampling_freq / (2.0 * M_PI);
        dout << "Coarse frequency offset compensation: "
            << current_foe_comp << " (" << foe_comp_hz << " Hz)" << std::endl;
    }

    // return start position of the L-STF (with guard interval)
    return frame_start_pos;
}

int
gnb_sync_impl::fine_sync(const gr_vector_const_void_star &input_items, int buffer_len,
                         float &current_foe_comp, float &fine_foe_comp, int &rx_ready_cnt, int &fine_foe_cnt)
{
    // compensate coarse FOE
    for (int i = 0; i < buffer_len; i++)
    {
        gr_complex comp_val = exp(gr_complex(0, current_foe_comp * (double) i));
        for (int ch = 0; ch < d_num_chans; ch++)
        {
            const gr_complex *in = (const gr_complex *) input_items[ch];
            d_input_buf[i + ch * XCORR_DATA_LEN] = in[i] * comp_val;
        }
    }

    // compute cross-correlation of received signal and L-LTF
    int xcorr_len = buffer_len - FFT_LEN;
    for (int ch = 0; ch < d_num_chans; ch++)
        d_xcorr_fir.filterN(&d_xcorr_buf[ch * MAX_XCORR_LEN], &d_input_buf[ch * XCORR_DATA_LEN], xcorr_len);


    // compute and validate xcorr peaks
    double max_xcorr = 0, avg_xcorr = 0;
    int peak_pos = -1;
    for (int i = 0; i < xcorr_len; i++)
    {
        float xcorr_norm = 0;
        for (int ch = 0; ch < d_num_chans; ch++)
            xcorr_norm += norm(d_xcorr_buf[MAX_XCORR_LEN * ch + i]);
        d_xcorr_val[i] = xcorr_norm;
        avg_xcorr += xcorr_norm;
        if (xcorr_norm > max_xcorr)
        {
            max_xcorr = xcorr_norm;
            peak_pos = i;
        }
    }
    avg_xcorr /= (double) xcorr_len;
    if (max_xcorr < 10.0 * avg_xcorr) // TODO fine-tune max_xcorr threshold
    {
        dout << "No valid xcorr peaks found (" << peak_pos << ")" << std::endl;
        return -1;
    }

    // scan for first and second xcorr peaks
    int deltaCSD = 4;
    float first_peak = 0, second_peak = 0;
    int first_peak_pos = -1, second_peak_pos = -1;
    for (int i = 0; i < xcorr_len; i++)
    {
        float xcorr_val = d_xcorr_val[i];
        if (xcorr_val < d_xcorr_thrd * max_xcorr)
            continue;

        if (first_peak_pos == -1)
        {
            // first peak not found yet
            first_peak = xcorr_val;
            first_peak_pos = i;
        }
        else if (i <= first_peak_pos + LTF_LEN / 4 + 5)
        {
            // within scope of first peak
            if (xcorr_val > first_peak)
            {
                first_peak = xcorr_val;
                first_peak_pos = i;
            }
        }
        else if (xcorr_val > second_peak)
        {
            // second peak scope
            second_peak = xcorr_val;
            second_peak_pos = i;
        }
    }
    dout << "Found LTF xcorr peaks at " << first_peak_pos << ", " << second_peak_pos << std::endl;

    // sanity checks
    int sig_start = -1;
    if (first_peak_pos > 0 && second_peak_pos >= first_peak_pos + FFT_LEN - deltaCSD &&
        second_peak_pos <= first_peak_pos + FFT_LEN + deltaCSD)
    {
        // start of the HT preambles (HT-SIGs)
        // sig_start = first_peak_pos - deltaCSD + 2 * FFT_LEN + SYM_LEN;

        // calculate the start of the HT-LTF (removing HT-SIG and HT-STF)
        // first peak pos corresponding to the start of the first FFT block in L-LTF
        // 4 * SYM_LEN : L-SIG, HT-SIG, HT-STF
        sig_start = first_peak_pos + 2 * FFT_LEN + 4 * SYM_LEN;
    }
    else
    {
        return sig_start;
    }

    // estimate fine FOE
    gr_complex corr_foe = 0;
    for (int k = first_peak_pos - 12; k < first_peak_pos + FFT_LEN; k++)
    {
        int delay_idx = k + FFT_LEN;
        for (int ch = 0; ch < d_num_chans; ch++)
            corr_foe += d_input_buf[ch * XCORR_DATA_LEN + k] * conj(d_input_buf[ch * XCORR_DATA_LEN + delay_idx]);
    }
    float cur_fine_foe_comp = arg(corr_foe) / (float) FFT_LEN; // FOE compensation in radians

    if (rx_ready_cnt < 10)
    {
        if (rx_ready_cnt % 2 == 0)
            fine_foe_comp = cur_fine_foe_comp;
        else
        {
            current_foe_comp += 0.1 * (fine_foe_comp + cur_fine_foe_comp);
            dout << "Fine frequency offset compensation: " << current_foe_comp << "    ("
                 << current_foe_comp * d_sampling_freq / (2.0 * M_PI) << " Hz)" << std::endl;
        }
    }
    else if (fine_foe_cnt < 10)
    {
        fine_foe_cnt += 1;
        fine_foe_comp += cur_fine_foe_comp;
    }
    else
    {
        fine_foe_cnt = 0;
        current_foe_comp += (fine_foe_comp + cur_fine_foe_comp) / 200.0;
        fine_foe_comp = 0.0;
        dout << "Fine frequency offset compensation: " << current_foe_comp << "    ("
             << current_foe_comp * d_sampling_freq / (2.0 * M_PI) << " Hz)" << std::endl;
    }

    // return start position of L-SIG
    return sig_start;
}

// conjugated and reversed LTF sequence
const
std::vector<gr_complex> gnb_sync_impl::LTF_SEQ = {
    gr_complex(-0.0455, -1.0679), gr_complex(0.3528, -0.9865), gr_complex(0.8594, 0.7348),
    gr_complex(0.1874, 0.2475), gr_complex(0.5309, -0.7784), gr_complex(-1.0218, -0.4897),
    gr_complex(-0.3401, -0.9423), gr_complex(0.8657, -0.2298), gr_complex(0.4734, 0.0362),
    gr_complex(0.0088, -1.0207), gr_complex(-1.2142, -0.4205), gr_complex(0.2172, -0.5195),
    gr_complex(0.5207, -0.1326), gr_complex(-0.1995, 1.4259), gr_complex(1.0583, -0.0363),
    gr_complex(0.5547, -0.5547), gr_complex(0.3277, 0.8728), gr_complex(-0.5077, 0.3488),
    gr_complex(-1.1650, 0.5789), gr_complex(0.7297, 0.8197), gr_complex(0.6173, 0.1253),
    gr_complex(-0.5353, 0.7214), gr_complex(-0.5011, -0.1935), gr_complex(-0.3110, -1.3392),
    gr_complex(-1.0818, -0.1470), gr_complex(-1.1300, -0.1820), gr_complex(0.6663, -0.6571),
    gr_complex(-0.0249, 0.4773), gr_complex(-0.8155, 1.0218), gr_complex(0.8140, 0.9396),
    gr_complex(0.1090, 0.8662), gr_complex(-1.3868, -0.0000), gr_complex(0.1090, -0.8662),
    gr_complex(0.8140, -0.9396), gr_complex(-0.8155, -1.0218), gr_complex(-0.0249, -0.4773),
    gr_complex(0.6663, 0.6571), gr_complex(-1.1300, 0.1820), gr_complex(-1.0818, 0.1470),
    gr_complex(-0.3110, 1.3392), gr_complex(-0.5011, 0.1935), gr_complex(-0.5353, -0.7214),
    gr_complex(0.6173, -0.1253), gr_complex(0.7297, -0.8197), gr_complex(-1.1650, -0.5789),
    gr_complex(-0.5077, -0.3488), gr_complex(0.3277, -0.8728), gr_complex(0.5547, 0.5547),
    gr_complex(1.0583, 0.0363), gr_complex(-0.1995, -1.4259), gr_complex(0.5207, 0.1326),
    gr_complex(0.2172, 0.5195), gr_complex(-1.2142, 0.4205), gr_complex(0.0088, 1.0207),
    gr_complex(0.4734, -0.0362), gr_complex(0.8657, 0.2298), gr_complex(-0.3401, 0.9423),
    gr_complex(-1.0218, 0.4897), gr_complex(0.5309, 0.7784), gr_complex(0.1874, -0.2475),
    gr_complex(0.8594, -0.7348), gr_complex(0.3528, 0.9865), gr_complex(-0.0455, 1.0679),
    gr_complex(1.3868, -0.0000),
};

} /* namespace gr::ncjt */
