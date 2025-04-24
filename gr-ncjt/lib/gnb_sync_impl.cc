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
// static const pmt::pmt_t RATE_KEY = pmt::string_to_symbol("rx_rate");
// static const pmt::pmt_t FREQ_KEY = pmt::string_to_symbol("rx_freq");

gnb_sync::sptr
gnb_sync::make(
    int nchans, double samplerate, int pktspersec, int hwdelayp2, int hwdelayp3,
    int p2_htlen, int p2_datalen, int p3_htlen, int p3_datalen, double p2_start, double p3_start,
    double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool debug)
{
    return gnuradio::make_block_sptr<gnb_sync_impl>(
        nchans, samplerate, pktspersec, hwdelayp2, hwdelayp3, p2_htlen, p2_datalen, p3_htlen, p3_datalen, p2_start, p3_start,
        rxpwr_thrd, acorr_thrd, xcorr_thrd, max_corr_len, debug);
}

gnb_sync_impl::gnb_sync_impl(
    int nchans, double samplerate, int pktspersec, int hwdelayp2, int hwdelayp3,
    int p2_htlen, int p2_datalen, int p3_htlen, int p3_datalen, double p2_start, double p3_start,
    double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool debug)
    : gr::block("gnb_sync",
                gr::io_signature::make(nchans, nchans, sizeof(gr_complex)),
                gr::io_signature::make(nchans, 2 * nchans, sizeof(gr_complex))),
      d_rxpwr_thrd(rxpwr_thrd), d_acorr_thrd(acorr_thrd), d_xcorr_thrd(xcorr_thrd), d_max_corr_len(max_corr_len),
      d_wait_interval0(0), d_wait_interval1(0), d_wait_interval2(0),
      d_xcorr_fir(gr::filter::kernel::fir_filter_ccc(LTF_SEQ)),
      d_debug(debug)
{
    if (nchans < 1 || nchans > MAX_CHANS)
        throw std::runtime_error("Currently support 1 to 8 IQ channels");
    d_num_chans = nchans;

    if (samplerate < 1e6 || samplerate > 1e8)
        throw std::runtime_error("invalid sampling frequency specified");
    d_samplerate = samplerate;
    if (pktspersec <= 0 || pktspersec > 100)
        throw std::runtime_error("invalid sampling packet transmission interval specified");

    // packet transmission cycles period in seconds
    d_pkt_interval = 1.0 / (double) pktspersec;
    d_frame_interval = (uint64_t) floor(d_samplerate / pktspersec);

    if (hwdelayp2 < 0 || hwdelayp2 > samplerate / 1000)
        throw std::runtime_error("invalid P2 USRP processing delay specified");
    d_hw_delay_p2 = hwdelayp2;
    if (hwdelayp3 < 0 || hwdelayp3 > samplerate / 1000)
        throw std::runtime_error("invalid P3 USRP processing delay specified");
    d_hw_delay_p3 = hwdelayp3;

    if (p2_htlen < 0 || (p2_htlen >= SYM_LEN * MAX_PREAMBLE_SYMS))
        throw std::runtime_error("invalid HT preamble length specified");
    d_p2_htlen = p2_htlen;
    if (p2_datalen < 0 || (p2_datalen > int(samplerate / pktspersec)))
        throw std::runtime_error("invalid data frame length specified");

    if (p3_htlen < 0 || (p3_htlen >= SYM_LEN * MAX_PREAMBLE_SYMS))
        throw std::runtime_error("invalid HT preamble length specified");
    d_p3_htlen = p3_htlen;
    if (p3_datalen < 0 || (p3_datalen > int(samplerate / pktspersec)))
        throw std::runtime_error("invalid data frame length specified");

    if (p2_start < 0 || p2_start > 0.5 * d_pkt_interval)
        throw std::runtime_error("invalid phase 2 start time specified");
    d_p2_offset = (uint64_t) floor(p2_start * samplerate);
    if (p3_start < p2_start || p3_start > d_pkt_interval)
        throw std::runtime_error("invalid phase 3 start time specified");
    d_p3_offset = (uint64_t) floor(p3_start * samplerate);

    // total length of HT-LTF and data symbols, excluding HT-SIG and HT-LTF (3*SYM_LEN)
    d_frame_len_1 = p2_htlen + p2_datalen - 3 * SYM_LEN;
    d_frame_len_2 = p3_htlen + p3_datalen - 3 * SYM_LEN;
    d_frame_start1 = 0;
    d_frame_start2 = 0;

    d_corr_buf_pos = 0;
    d_corr_buf = malloc_complex(CORR_BUF_LEN);
    d_pwrest_buf = malloc_float(CORR_BUF_LEN);

    d_input_buf = malloc_complex(XCORR_DATA_LEN * nchans);
    d_xcorr_buf = malloc_complex(MAX_XCORR_LEN * nchans);
    d_xcorr_val = malloc_float(1024);

    d_state = RXTIME;
    d_rxtime_offset = 0;
    d_wait_count = 0;
    d_data_samples = 0;
    d_rx_ready_cnt1 = 0;
    d_rx_ready_cnt2 = 0;
    d_sync_err_cnt1 = 0;
    d_sync_err_cnt2 = 0;
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
        case P2SYNC:
        case P3SYNC:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * LTF_LEN;
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
    int noutput_samples;

    switch (d_state)
    {
        case RXTIME:
        {
            // send tag command to check current timestamp
            send_tagcmd();

            if (d_rxtime_offset <= 0)
                check_rxtime(min_input_items);

            if (d_rxtime_offset > 0)
            {
                // skip first 10 seconds for gNB receiver
                if (nitems_read(0) < uint64_t(10 * d_samplerate))
                {
                    d_state = RXTIME;
                    consume_each(min_input_items);
                    break;
                }

                // Adjust frame start according to d_rxtime_offset
                uint64_t cur_frame_pos = nitems_read(0);
                uint64_t next_frame_pos = cur_frame_pos + d_frame_interval - (cur_frame_pos % d_frame_interval);

                uint64_t start_offset = uint64_t(d_samplerate) - d_rxtime_offset;
                d_frame_start1 = next_frame_pos + uint64_t(start_offset + d_p2_offset + d_hw_delay_p2);
                d_frame_start2 = next_frame_pos + uint64_t(start_offset + d_p3_offset + d_hw_delay_p3);

                // calculate sample clocks until next P2 transmission
                d_wait_interval0 = int(d_frame_start1 - cur_frame_pos) - d_max_corr_len / 4;
                d_wait_count = 0;
                d_state = WAIT0;
                dout << "Adjusting P2 start: " << d_frame_start1 << std::endl;
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
                d_fine_foe_cnt1 = 0;
                d_fine_foe_cnt2 = 0;
                d_fine_foe_comp1 = 0.0;
                d_fine_foe_comp2 = 0.0;
                d_current_foe_comp1 = 0.0;
                d_current_foe_comp2 = 0.0;
                d_rx_ready_cnt1 = 0;
                d_rx_ready_cnt2 = 0;
                d_sync_err_cnt1 = 0;
                d_sync_err_cnt2 = 0;
                d_state = P2SEARCH;
                dout << "Phase 2: Entering SYNC state" << std::endl;
            }
            consume_each(min_input_items);
            break;
        }
        case P2SEARCH:
        {
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 10 * STF_LEN)
                break;

            // search for frame start
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp1);
            if (start_pos < 0)
            {
                dout << "Phase 2: Packet start not detected correctly: " << start_pos << std::endl;
                d_current_foe_comp1 = 0.0;
                d_skip_p2_frame = true;
                // skip the search buffer
                int skip_samples = d_max_corr_len / 4;
                consume_each(skip_samples);
                d_data_samples = 0;
                d_state = P2DEFRAME;
                break;
            }
            else
            {
                d_state = P2SYNC;
                consume_each(start_pos);
                dout << "Phase 2: Packet start detected at " << start_pos << std::endl;
            }
            break;
        }
        case P2SYNC:
        {
            int ht_start = fine_sync(input_items, 3 * LTF_LEN,
                                     d_current_foe_comp1, d_fine_foe_comp1,
                                     d_rx_ready_cnt1, d_fine_foe_cnt1);
            if (ht_start < 0)
            {
                dout << "Phase 2: Fine synchronize failed for TX UE!" << std::endl;
                d_sync_err_cnt1 += 1;
                if (d_sync_err_cnt1 > 5)
                {
                    d_skip_p2_frame = true;
                    d_current_foe_comp1 = 0.0;
                    d_rx_ready_cnt1 = 0;
                    d_sync_err_cnt1 = 0;
                }
                // skip the whole beacon
                consume_each(5 * SYM_LEN + d_p2_htlen);
                d_data_samples = 0;
                d_state = P2DEFRAME;
                break;
            }

            uint64_t frame_start = nitems_read(0) + ht_start;
            dout << "Phase 2: Frame start at " << ht_start << " (" << frame_start << ")" << std::endl;
            consume_each(ht_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            d_data_samples = 0;
            d_skip_p2_frame = false;
            d_state = P2DEFRAME;

            if (d_rx_ready_cnt1 > 10 && (d_rx_ready_cnt1 % 100) == 0)
            {
                std::cout << "Phase 2: Fine frequency offset compensation: " << d_current_foe_comp1 << "    ("
                          << d_current_foe_comp1 * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            d_rx_ready_cnt1 += 1;
            if (d_rx_ready_cnt1 >= 10 && d_rx_ready_cnt2 >= 10) // changing to ready state
                send_rxstate(true);
            else
                send_rxstate(false);

            break;
        }
        case P2DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len_1)
            {
                noutput_samples = d_frame_len_1 - d_data_samples;
                d_frame_start1 += d_frame_interval;
                d_wait_interval1 = int(d_frame_start2 - nitems_read(0)) - noutput_samples;
                if (d_rx_ready_cnt2 == 0)
                    d_wait_interval1 -= d_max_corr_len / 4;
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
                                 pmt::from_long(d_frame_len_1),
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
                d_state =  (d_rx_ready_cnt2 == 0) ? P3SEARCH : P3SYNC;
                dout << "Phase 3: Entering SYNC state" << std::endl;
            }
            consume_each(min_input_items);
            break;
        }
        case P3SEARCH:
        {
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 10 * STF_LEN)
                break;

            // search for frame start
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp2);
            if (start_pos < 0)
            {
                dout << "Phase 3: Packet start not detected correctly: " << start_pos << std::endl;
                d_current_foe_comp2 = 0.0;
                d_skip_p3_frame = true;
                // skip the search buffer
                int skip_samples = d_max_corr_len / 4;
                consume_each(skip_samples);
                d_data_samples = 0;
                d_state = P3DEFRAME;
                break;
            }
            else
            {
                d_state = P3SYNC;
                consume_each(start_pos);
                dout << "Phase 3: Packet start detected at " << start_pos << std::endl;
            }

            break;
        }
        case P3SYNC:
        {
            int ht_start = fine_sync(input_items, 3 * LTF_LEN,
                                     d_current_foe_comp2, d_fine_foe_comp2,
                                     d_rx_ready_cnt2, d_fine_foe_cnt2);
            if (ht_start < 0)
            {
                dout << "Phase 3: Fine synchronize failed for RX UE!" << std::endl;
                d_sync_err_cnt2 += 1;
                if (d_sync_err_cnt2 > 5)
                {
                    d_skip_p3_frame = true;
                    d_current_foe_comp2 = 0.0;
                    d_rx_ready_cnt2 = 0;
                    d_sync_err_cnt2 = 0;
                }
                d_current_foe_comp2 = 0.0;
                // skip the whole beacon
                consume_each(5 * SYM_LEN + d_p3_htlen);
                d_data_samples = 0;
                d_state = P3DEFRAME;
                break;
            }

            uint64_t frame_start = nitems_read(0) + ht_start;
            dout << "Phase 3: Frame start at " << ht_start << " (" << frame_start << ")" << std::endl;
            consume_each(ht_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            d_data_samples = 0;
            d_skip_p3_frame = false;
            d_state = P3DEFRAME;

            if (d_rx_ready_cnt2 > 10 && (d_rx_ready_cnt2 % 100) == 0)
            {
                std::cout << "Phase 3: Fine frequency offset compensation: " << d_current_foe_comp2 << "    ("
                          << d_current_foe_comp2 * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            d_rx_ready_cnt2 += 1;
            if (d_rx_ready_cnt1 >= 10 && d_rx_ready_cnt2 >= 10) // changing to ready state
                send_rxstate(true);
            else
                send_rxstate(false);

            break;
        }
        case P3DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len_2)
            {
                noutput_samples = d_frame_len_2 - d_data_samples;
                d_frame_start2 += d_frame_interval;
                d_wait_interval2 = int(d_frame_start1 - nitems_read(0)) - noutput_samples;
                if (d_rx_ready_cnt1 == 0)
                    d_wait_interval2 -= d_max_corr_len / 4;
                d_wait_count = 0;
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
                for (int ch = 0; ch < d_num_chans; ch++)
                    add_item_tag(d_num_chans + ch,
                                 nitems_written(d_num_chans + ch),
                                 pmt::string_to_symbol("frame_start"),
                                 pmt::from_long(d_frame_len_2),
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
                d_data_samples = 0;
                d_state = (d_rx_ready_cnt1 == 0) ? P2SEARCH : P2SYNC;
                dout << "Phase 2: Entering SYNC state" << std::endl;
            }
            consume_each(min_input_items);
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
gnb_sync_impl::check_rxtime(int rx_windows_size)
{
    // Search for rx_time tag
    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, rx_windows_size, TIME_KEY);
    if (!d_tags.empty())
    {
        // extract integer and fractional timestamp
        auto pt = pmt::to_tuple(d_tags[0].value);
        uint64_t time_secs = pmt::to_uint64(pmt::tuple_ref(pt, 0));
        double time_fracs = pmt::to_double(pmt::tuple_ref(pt, 1));
        double tag_time = double(time_secs) + time_fracs;

        if (d_rxtime_offset <= 0)
        {
            d_rxtime_offset = int64_t(uint64_t(tag_time * d_samplerate) - d_tags[0].offset);
            std::cout << "Tag time: " << time_secs << ":" << time_fracs
                      << "  rxtime offset: " << double(d_rxtime_offset) / d_samplerate
                      << "  (" << d_rxtime_offset << ")" << std::endl;
        }
        else
        {
            int64_t rxtime_offset = int64_t(uint64_t(tag_time * d_samplerate) - d_tags[0].offset);
            double time_drift = abs(rxtime_offset - d_rxtime_offset);
            d_rxtime_offset = rxtime_offset;
            if (time_drift >= 1)
            {
                std::cout << "Tag time: " << time_secs << ":" << time_fracs
                          << "  rxtime offset: " << double(d_rxtime_offset) / d_samplerate
                          << "  (" << d_rxtime_offset << ")" << std::endl;
                d_rxtime_offset = 0;
            }
        }
    }
}

/*
 * search for frame start using L-STF auto-corr peaks
 */
int
gnb_sync_impl::sync_search(const gr_vector_const_void_star &input_items, int buffer_len, float &current_foe_comp)
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

        // check auto-corr peaks, use noise lower-bound to avoid false detection
        float corr_norm = norm(auto_corr) / (power_est * power_est + 1e-6f);
        if (power_est > d_rxpwr_thrd && corr_norm >= d_acorr_thrd)
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
        if (peak_duration >= PEAK_THRD * CORR_DELAY)
        {
            // frame start should be within CORR_DELAY samples before the start of L-STF
            frame_start_pos = peak_start - (PEAK_THRD - 2) * CORR_DELAY;
            dout << "Auto-correlation peak found at " << peak_start << " (peak duration " << peak_duration << ")"
                 << std::endl;
            break;
        }
    }

    if (frame_start_pos >= 0)
    {
        gr_complex corr_foe = 0;
        // avoid the initial/last CORR_DELAY samples, 8*CORR_DELAY samples are usable
        for (int k = frame_start_pos + CORR_DELAY; k < frame_start_pos + 9 * CORR_DELAY; k++)
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
        float foe_comp_hz = current_foe_comp * d_samplerate / (2.0 * M_PI);
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
    // compensate for current FOE
    float sig_power = 0.0;
    for (int i = 0; i < buffer_len; i++)
    {
        gr_complex comp_val = exp(gr_complex(0, current_foe_comp * (double) i));
        for (int ch = 0; ch < d_num_chans; ch++)
        {
            auto *in = (const gr_complex *) input_items[ch];
            sig_power += norm(in[i]);
            d_input_buf[i + ch * XCORR_DATA_LEN] = in[i] * comp_val;
        }
    }
    sig_power /= buffer_len;

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
    // if (max_xcorr < 8.0 * avg_xcorr) // TODO fine-tune max_xcorr threshold
    if (max_xcorr < 8.0 * sig_power) // TODO fine-tune max_xcorr threshold
    {
        dout << "Xcorr mean: " << avg_xcorr << "  Xcorr peak: " << max_xcorr << std::endl;
        dout << "No valid xcorr peaks found (" << peak_pos << ")" << std::endl;
        return -1;
    }

    // scan for first and second xcorr peaks
    float first_peak = 0, second_peak = 0;
    int first_peak_pos = -1, second_peak_pos = -1;
    for (int i = LTF_LEN; i < xcorr_len - FFT_LEN; i++)
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
        else if (i <= first_peak_pos + FFT_LEN / 4)
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
    int deltaCSD = 4;
    int ht_start = -1;
    if (first_peak_pos > 0 && second_peak_pos >= first_peak_pos + FFT_LEN - deltaCSD &&
        second_peak_pos <= first_peak_pos + FFT_LEN + deltaCSD)
    {
        // start of the HT-LTF signals
        // ht_start = first_peak_pos - deltaCSD + 2 * FFT_LEN + SYM_LEN;

        // calculate the start of the HT-LTF (removing HT-SIG and HT-STF)
        // first peak pos corresponding to the start of the first FFT block in L-LTF
        // 4 * SYM_LEN : L-SIG, HT-SIG, HT-STF
        ht_start = first_peak_pos + 2 * FFT_LEN + 4 * SYM_LEN;
    }
    else
    {
        return ht_start;
    }

    // estimate fine FOE
    gr_complex corr_foe = 0;
    for (int k = first_peak_pos - 2 * deltaCSD; k < first_peak_pos + FFT_LEN - 2 * deltaCSD; k++)
    {
        int delay_idx = k + FFT_LEN;
        for (int ch = 0; ch < d_num_chans; ch++)
            corr_foe += d_input_buf[ch * XCORR_DATA_LEN + k] * conj(d_input_buf[ch * XCORR_DATA_LEN + delay_idx]);
    }
    float cur_fine_foe_comp = arg(corr_foe) / (float) FFT_LEN; // FOE compensation in radians

    if (rx_ready_cnt < 20)
    {
        if (rx_ready_cnt % 2 == 0)
            fine_foe_comp = cur_fine_foe_comp;
        else
        {
            current_foe_comp += 0.1 * (fine_foe_comp + cur_fine_foe_comp);
            dout << "Fine frequency offset compensation: " << current_foe_comp << "    ("
                 << current_foe_comp * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
        }
    }
    else if (fine_foe_cnt < 20)
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
             << current_foe_comp * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
    }

    // return start position of HT-LTF
    return ht_start;
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
