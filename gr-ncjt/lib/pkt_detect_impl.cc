/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "pkt_detect_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <volk/volk.h>
#include "utils.h"

namespace gr::ncjt
{

static const pmt::pmt_t TIME_KEY = pmt::string_to_symbol("rx_time");
static const pmt::pmt_t RATE_KEY = pmt::string_to_symbol("rx_rate");
static const pmt::pmt_t FREQ_KEY = pmt::string_to_symbol("rx_freq");

pkt_detect::sptr
pkt_detect::make(int nchans, int preamblelen, int dataframelen, double samplerate, int pktspersec,
                 double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool sync_all, bool debug)
{
    return gnuradio::make_block_sptr<pkt_detect_impl>(
        nchans, preamblelen, dataframelen, samplerate, pktspersec,
        rxpwr_thrd, acorr_thrd, xcorr_thrd, max_corr_len, sync_all, debug);
}

pkt_detect_impl::pkt_detect_impl(int nchans, int preamblelen, int dataframelen, double samplerate, int pktspersec,
                                 double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool sync_all, bool debug)
    : gr::block("pkt_detect",
                gr::io_signature::make(nchans, nchans, sizeof(gr_complex)),
                gr::io_signature::make(nchans, nchans, sizeof(gr_complex))),
      d_sampling_freq(samplerate), d_rxpwr_thrd(rxpwr_thrd), d_acorr_thrd(acorr_thrd),d_xcorr_thrd(xcorr_thrd),
      d_max_corr_len(max_corr_len), d_rx_ready_cnt(0), d_sync_all(sync_all), d_fine_foe_cnt(1),
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

    // total length of HT-LTF and data symbols, excluding HT-SIG and HT-LTF (3*SYM_LEN)
    d_frame_len = dataframelen + preamblelen - 3 * SYM_LEN;

    d_pkt_interval = 1.0 / (double) pktspersec;
    // assuming legacy preamble of 5 symbol length (L-STF, L-LTF, and L-SIG) and 3 symbols of HT-SIG and HT-LTF
    d_wait_interval = (int) floor(samplerate * d_pkt_interval) - d_frame_len - 8 * SYM_LEN;

    d_corr_buf_pos = 0;
    d_corr_buf = malloc_complex(CORR_BUF_LEN);
    d_pwrest_buf = malloc_float(CORR_BUF_LEN);

    d_input_buf = malloc_complex(XCORR_DATA_LEN * nchans);
    d_xcorr_buf = malloc_complex(MAX_XCORR_LEN * nchans);
    d_xcorr_val = malloc_float(1024);

    d_state = SEARCH;
    d_wait_count = 0;
    d_rx_demod_en = false;
    d_rx_all_sync = false;
    d_rx_all_ready = false;

    // message_port_register_out(pmt::mp("uhdcmd"));
    // message_port_register_out(pmt::mp("rxtime"));
    message_port_register_out(pmt::mp("rxrdy"));

    message_port_register_in(pmt::mp("allrdy"));
    set_msg_handler(pmt::mp("allrdy"), [this](const pmt::pmt_t &msg) { process_allsync_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());
}

pkt_detect_impl::~pkt_detect_impl()
{
    volk_free(d_corr_buf);
    volk_free(d_pwrest_buf);
    volk_free(d_input_buf);
    volk_free(d_xcorr_buf);
    volk_free(d_xcorr_val);
}

void
pkt_detect_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    switch (d_state)
    {
        case SEARCH:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * LTF_LEN;
            break;
        }
        case FINESYNC:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 4 * LTF_LEN;
            break;
        }
        case WAIT:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * SYM_LEN;
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
pkt_detect_impl::process_allsync_message(const pmt::pmt_t &msg)
{
    bool all_sync = pmt::to_bool(msg);
    if (all_sync)
        d_rx_all_sync = true;
}

void
pkt_detect_impl::send_rxstate(bool ready) {
    pmt::pmt_t dict = pmt::make_dict();
    dict = pmt::dict_add(dict, pmt::mp("rxrdy"), _id);
    message_port_pub(pmt::mp("rxrdy"), dict);
}

int
pkt_detect_impl::general_work(int noutput_items,
                              gr_vector_int &ninput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_num_chans; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int noutput_samples = 0;

    switch (d_state)
    {
        case SEARCH:
        {
            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len);
            if (start_pos < 0)
            {
                // frame start not found, discard samples
                consume_each(buffer_len - 2 * STF_LEN);
                break;
            }
            d_frame_start = nitems_read(0) + start_pos;
            dout << "Packet start detected at " << d_frame_start << std::endl;
            consume_each(start_pos);
            d_state = FINESYNC;
            break;
        }
        case FINESYNC:
        {
            // int input_len = min(min_input_items, 3 * LTF_LEN);
            if (min_input_items < 3 * LTF_LEN)
                break;

            int htsig_start = fine_sync(input_items, 3 * LTF_LEN);
            if (htsig_start < 0)
            {
                dout << "Fine synchronize failed!" << std::endl;
                consume_each(min_input_items);
                d_rx_ready_cnt = 0;
                d_state = SEARCH;
                break;
            }
            d_frame_start = nitems_read(0) + htsig_start;
            dout << "Data frame start at " << d_frame_start << std::endl;
            d_data_samples = 0;
            d_state = DEFRAME;

            consume_each(htsig_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            if (d_rx_ready_cnt < 100)
                d_rx_ready_cnt += 1;
            break;
        }
        case DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len)
            {
                noutput_samples = d_frame_len - d_data_samples;
                d_state = WAIT;
            }

            // skip initial frames with unstable FOE
            // wait for all other receiver all synced
            if (d_rx_ready_cnt <= 20 || (d_sync_all && !d_rx_all_sync))
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
                {
                    add_item_tag(ch,
                                 nitems_written(0),
                                 pmt::string_to_symbol("packet_len"),
                                 pmt::from_long(d_frame_len),
                                 pmt::string_to_symbol(name()));

                    add_item_tag(ch,
                                 nitems_written(0),
                                 pmt::string_to_symbol("frame_start"),
                                 pmt::from_long(d_frame_len),
                                 pmt::string_to_symbol(name()));
                }
            }

            for (int i = 0; i < noutput_samples; i++)
            {
                gr_complex comp_val = exp(gr_complex(0, d_current_foe_comp * d_data_samples));
                for (int ch = 0; ch < d_num_chans; ch++)
                {
                    auto in = (const gr_complex *) input_items[ch];
                    auto out = (gr_complex *) output_items[ch];
                    out[i] = in[i] * comp_val;
                }
                d_data_samples += 1;
            }

            consume_each(noutput_samples);
            break;
        }
        case WAIT:
        {
            if (!d_rx_demod_en && d_rx_ready_cnt >= 20)
            {
                d_rx_demod_en = true;
                send_rxstate(true);
                std::cout << "Synchronization acquired" << std::endl;
                std::cout << "Fine frequency offset compensation: " << d_current_foe_comp << "    ("
                          << d_current_foe_comp * d_sampling_freq / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            d_wait_count += min_input_items;
            int new_data_count = d_wait_count - d_wait_interval;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                d_state = FINESYNC; // SEARCH;
                if (d_rx_demod_en && d_rx_all_sync)
                    d_rx_all_ready = true;
            }
            consume_each(min_input_items);
            break;
        }
    }

    return noutput_samples;
}

/*
 * search for frame start using L-LTS auto-corr peaks
 */
int
pkt_detect_impl::sync_search(const gr_vector_const_void_star &input_items, int buffer_len)
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
        d_current_foe_comp = arg(corr_foe) / (float) CORR_DELAY; // FOE compensation in radians
        float foe_comp_hz = d_current_foe_comp * d_sampling_freq / (2.0 * M_PI);
        dout << "Coarse frequency offset compensation: "
             << d_current_foe_comp << " (" << foe_comp_hz << " Hz)" << std::endl;
    }

    // return start position of the L-STF (with guard interval)
    return frame_start_pos;
}

int
pkt_detect_impl::fine_sync(const gr_vector_const_void_star &input_items, int buffer_len)
{
    // compensate for current FOE
    for (int i = 0; i < buffer_len; i++)
    {
        gr_complex comp_val = exp(gr_complex(0, d_current_foe_comp * (double) i));
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
        // start of the HT-LTF
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
    float fine_foe_comp = arg(corr_foe) / (float) FFT_LEN; // FOE compensation in radians

    if (d_rx_ready_cnt < 10)
    {
        if (d_rx_ready_cnt % 2 == 0)
            d_fine_foe_comp = fine_foe_comp;
        else
        {
            d_current_foe_comp += 0.1 * (d_fine_foe_comp + fine_foe_comp);
            dout << "Fine frequency offset compensation: " << d_current_foe_comp << "    ("
                 << d_current_foe_comp * d_sampling_freq / (2.0 * M_PI) << " Hz)" << std::endl;
        }
    }
    else if (d_fine_foe_cnt < 10)
    {
        d_fine_foe_cnt += 1;
        d_fine_foe_comp += fine_foe_comp;
    }
    else
    {
        d_fine_foe_cnt = 0;
        d_current_foe_comp += (d_fine_foe_comp + fine_foe_comp) / 200.0;
        d_fine_foe_comp = 0.0;
        dout << "Fine frequency offset compensation: " << d_current_foe_comp << "    ("
             << d_current_foe_comp * d_sampling_freq / (2.0 * M_PI) << " Hz)" << std::endl;
    }

    // return start position of L-SIG
    return sig_start;
}

// conjugated and reversed LTF sequence
const
std::vector<gr_complex> pkt_detect_impl::LTF_SEQ = {
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
