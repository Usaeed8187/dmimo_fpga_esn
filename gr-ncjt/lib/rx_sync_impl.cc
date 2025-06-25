/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "rx_sync_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <iomanip>
#include <volk/volk.h>
#include "utils.h"

namespace gr::ncjt
{

static const pmt::pmt_t TIME_KEY = pmt::string_to_symbol("rx_time");
// static const pmt::pmt_t RATE_KEY = pmt::string_to_symbol("rx_rate");
// static const pmt::pmt_t FREQ_KEY = pmt::string_to_symbol("rx_freq");

rx_sync::sptr
rx_sync::make(int nchans, bool p1rx, int preamblelen, int dataframelen, double samplerate, int pktspersec,
              bool p2rx, int hwdelayp2, int p2preamblelen, int p2framelen, double p2_start,
              bool p3rx, int hwdelayp3, int p3preamblelen, int p3framelen, double p3_start, double rxpwr_thrd,
              double acorr_thrd, double xcorr_thrd, int max_corr_len, bool lltf2, bool debug)
{
    return gnuradio::make_block_sptr<rx_sync_impl>(
        nchans, p1rx, preamblelen, dataframelen, samplerate, pktspersec,
        p2rx, hwdelayp2, p2preamblelen, p2framelen, p2_start, p3rx, hwdelayp3, p3preamblelen, p3framelen, p3_start,
        rxpwr_thrd, acorr_thrd, xcorr_thrd, max_corr_len, lltf2, debug);
}

rx_sync_impl::rx_sync_impl(int nchans, bool p1rx, int preamblelen, int dataframelen, double samplerate, int pktspersec,
                           bool p2rx, int hwdelayp2, int p2preamblelen, int p2framelen, double p2_start,
                           bool p3rx, int hwdelayp3, int p3preamblelen, int p3framelen, double p3_start, double rxpwr_thrd,
                           double acorr_thrd, double xcorr_thrd, int max_corr_len, bool lltf2, bool debug)
    : gr::block("rx_sync",
                gr::io_signature::make(nchans, nchans, sizeof(gr_complex)),
                gr::io_signature::make(nchans, 3 * nchans, sizeof(gr_complex))),
      d_use_lltf2(lltf2), d_rxpwr_thrd(rxpwr_thrd), d_acorr_thrd(acorr_thrd), d_xcorr_thrd(xcorr_thrd), d_max_corr_len(max_corr_len),
      d_xcorr_fir_1(gr::filter::kernel::fir_filter_ccc(LTF_SEQ_1)),
      d_xcorr_fir_2(gr::filter::kernel::fir_filter_ccc(LTF_SEQ_2)), d_debug(debug)
{
    if (nchans < 1 || nchans > MAX_CHANS)
        throw std::runtime_error("Currently support 1 to 20 IQ channels");
    d_num_chans = nchans;

    if (samplerate < 10000 || samplerate > 100000000)
        throw std::runtime_error("invalid sampling frequency specified");
    if (pktspersec <= 0 || pktspersec > 100)
        throw std::runtime_error("invalid packet transmission interval specified");
    d_samplerate = samplerate;

    // packet frame burst transmission period in seconds
    d_frame_interval = (int) floor(samplerate / pktspersec);

    // reception enable flags for P1/P2/P3 phases
    d_p1rx = p1rx;
    d_p2rx = p2rx;
    d_p3rx = p3rx;

    // total length of HT/HT-LTF and data symbols, excluding HT/HE-SIG and HT/HE-STF
    d_frame_len1 = dataframelen;
    if (preamblelen < 0 || preamblelen >= SYM_LEN * MAX_PREAMBLE_SYMS)
        throw std::runtime_error("invalid HT preamble length specified");
    if (preamblelen < 0 || preamblelen > int(samplerate / pktspersec))
        throw std::runtime_error("invalid data frame length specified");
    d_ht_len1 = preamblelen;

    // Phase-2 frame params
    d_frame_len2 = p2framelen;
    if (p2preamblelen < 0 || p2preamblelen >= SYM_LEN * MAX_PREAMBLE_SYMS)
        throw std::runtime_error("invalid HT preamble length specified");
    if (p2preamblelen < 0 || p2preamblelen > int(samplerate / pktspersec))
        throw std::runtime_error("invalid data frame length specified");
    d_ht_len2 = p2preamblelen;
    d_p2start_offset = (uint64_t) floor(p2_start * samplerate);
    d_skip_p2_frame = false;
    d_hw_delay_p2 = hwdelayp2;

    // Phase-3 frame params
    d_frame_len3 = p3framelen;
    if (p3preamblelen < 0 || p3preamblelen >= SYM_LEN * MAX_PREAMBLE_SYMS)
        throw std::runtime_error("invalid HT preamble length specified");
    if (p3preamblelen < 0 || p3preamblelen > int(samplerate / pktspersec))
        throw std::runtime_error("invalid data frame length specified");
    d_ht_len3 = p3preamblelen;
    d_p3start_offset = (uint64_t) floor(p3_start * samplerate);
    d_skip_p3_frame = false;
    d_hw_delay_p3 = hwdelayp3;

    d_corr_buf_pos = 0;
    d_corr_buf = malloc_complex(CORR_BUF_LEN);
    d_pwrest_buf = malloc_float(CORR_BUF_LEN);
    d_input_buf = malloc_complex(XCORR_DATA_LEN * nchans);
    d_xcorr_buf = malloc_complex(MAX_XCORR_LEN * nchans);
    d_xcorr_val = malloc_float(1024);

    d_state = P1SEARCH;
    d_wait_interval = 0;
    d_wait_count = 0;
    d_data_samples = 0;
    d_fine_foe_comp1 = 0.0;
    d_fine_foe_cnt1 = 0;
    d_current_foe_comp1 = 0.0;
    d_fine_foe_comp2 = 0.0;
    d_fine_foe_cnt2 = 0;
    d_current_foe_comp2 = 0.0;
    d_fine_foe_comp3 = 0.0;
    d_fine_foe_cnt3 = 0;
    d_current_foe_comp3 = 0.0;
    d_rxtime_offset = 0;
    d_clk_offset_ok = false;
    d_clk_offset_cnt = 0;
    d_clk_offset_sum = 0;
    d_clk_offset_est = 0.0;

    d_rx_ready = false;
    d_rx_ready_cnt1 = 0;
    d_rx_ready_cnt2 = 0;
    d_rx_ready_cnt3 = 0;
    d_sync_err_cnt1 = 0;
    d_sync_err_cnt2 = 0;
    d_sync_err_cnt3 = 0;
    d_next_p1frame_start = 0;
    d_next_p2frame_start = 0;
    d_next_p3frame_start = 0;
    d_prev_p1frame_start = 0;
    d_prev_p2frame_start = 0;
    d_prev_p3frame_start = 0;

    message_port_register_out(pmt::mp("uhdcmd"));
    message_port_register_out(pmt::mp("rxtime"));
    message_port_register_out(pmt::mp("rxrdy"));

    set_tag_propagation_policy(block::TPP_DONT);

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());
}

rx_sync_impl::~rx_sync_impl()
{
    volk_free(d_corr_buf);
    volk_free(d_pwrest_buf);
    volk_free(d_input_buf);
    volk_free(d_xcorr_buf);
    volk_free(d_xcorr_val);
}

void
rx_sync_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    switch (d_state)
    {
        case P1SEARCH:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * LTF_LEN;
            break;
        }
        case P2SEARCH:
        {
            for (int ch = 0; ch < d_num_chans; ch++)
                ninput_items_required[ch] = 16 * LTF_LEN;
            break;
        }
        case P3SEARCH:
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

int
rx_sync_impl::general_work(int noutput_items, gr_vector_int &ninput_items,
                           gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_num_chans; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int noutput_samples = 0;

    switch (d_state)
    {
        case P1SEARCH:
        {
            // send tag command and check current timestamp
            if (d_rxtime_offset <= 0)
                send_tagcmd();

            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp1);
            if (start_pos < 0)
            {
                // frame start not found, discard samples
                consume_each(buffer_len - 2 * STF_LEN);
                break;
            }
            auto packet_start = nitems_read(0) + (uint64_t) start_pos;
            dout << "Packet start detected at " << packet_start << std::endl;
            consume_each(start_pos);
            d_rx_ready = false;
            d_sync_err_cnt1 = 0;
            d_sync_err_cnt2 = 0;
            d_sync_err_cnt3 = 0;
            d_clk_offset_sum = 0;
            d_clk_offset_cnt = 0;
            d_clk_offset_est = 0.0;
            d_prev_p1frame_start = 0;
            d_prev_p2frame_start = 0;
            d_prev_p3frame_start = 0;
            d_state = P1FINESYNC;
            break;
        }
        case P1FINESYNC:
        {
            // send tag command and check current timestamp
            if (d_rxtime_offset <= 0)
                send_tagcmd();

            if (min_input_items < 3 * LTF_LEN)
                break;

            // fine_sync return the start of the first HT-LTF field
            int ht_start = fine_sync(input_items, 3 * LTF_LEN, d_use_lltf2,
                                     d_current_foe_comp1, d_fine_foe_comp1,
                                     d_rx_ready_cnt1, d_fine_foe_cnt1);

            uint64_t cur_frame_start = 0;
            if (ht_start < 0)
            {
                dout << "Fine synchronize failed!" << std::endl;
                consume_each(min_input_items);
                d_sync_err_cnt1 += 1;
                if (d_sync_err_cnt1 >= 5)
                {
                    // changing from ready state
                    // std::cout << "======== Receiver synchronization lost ========" << std::endl;
                    send_rxstate(false);
                    d_sync_err_cnt1 = 0;
                    d_rx_ready_cnt1 = 0;
                    d_state = P1SEARCH;
                }
                break;
            }
            else
            {
                // skip HT preambles excluding LTFs
                ht_start += d_ht_len1;
                cur_frame_start = nitems_read(0) + ht_start;
                dout << "Data frame start at " << ht_start << " (" << cur_frame_start << ")" << std::endl;
            }

            if (d_rx_ready_cnt1 > 20 && (d_rx_ready_cnt1 % 100) == 0)
            {
                std::cout << "Fine frequency offset compensation: " << d_current_foe_comp1 << "    ("
                          << d_current_foe_comp1 * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            int64_t frame_offset = int64_t(cur_frame_start - d_prev_p1frame_start) - d_frame_interval;
            if (ht_start <= 0 || (d_prev_p1frame_start > 0 && (frame_offset < -SYM_LEN || frame_offset > SYM_LEN)))
            {
                // use frame start prediction
                cur_frame_start = d_prev_p1frame_start + uint64_t(d_frame_interval + d_clk_offset_est);
                ht_start = std::max(0, int(cur_frame_start - nitems_read(0)));
                std::cout << "Fine frame synchronization not correct, using prediction instead!" << std::endl;
            }
            else if (d_prev_p1frame_start > 0) // frame_offset >= -SYM_LEN && frame_offset <= SYM_LEN
            {
                // clock offset estimation
                d_clk_offset_sum += frame_offset;
                d_clk_offset_cnt += 1;
                if (d_clk_offset_cnt == CLK_EST_SAMPLES)
                {
                    d_clk_offset_est = (double) d_clk_offset_sum / (double) (d_clk_offset_cnt);
                    d_clk_offset_sum = 0;
                    d_clk_offset_cnt = 0;
                    d_clk_offset_ok = true;
                    double clk_offset_pps = 1e6 * d_clk_offset_est / (double) (d_frame_interval);
                    std::cout << "Fine clock offset estimation: " << d_clk_offset_est
                              << " (" << clk_offset_pps << " ppm)" << std::endl;
                }
            }

            // predict the next frame start positions
            if (d_rx_ready_cnt2 == 0 || d_prev_p2frame_start == 0)
                d_next_p2frame_start = cur_frame_start + d_p2start_offset - d_ht_len2 - 5 * SYM_LEN + d_hw_delay_p2;
            if (d_rx_ready_cnt3 == 0 || d_prev_p3frame_start == 0)
                d_next_p3frame_start = cur_frame_start + d_p3start_offset - d_ht_len3 - 5 * SYM_LEN + d_hw_delay_p3;
            d_next_p1frame_start = cur_frame_start + d_frame_interval;
            d_prev_p1frame_start = cur_frame_start;

            d_data_samples = 0;
            d_state = P1DEFRAME;

            if (d_rxtime_offset <= 0)
                check_rxtime(min_input_items);

            consume_each(ht_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF

            // if (d_rx_ready_cnt1 == 20) // changing to ready state
            // std::cout << "======== Receiver synchronization acquired ========" << std::endl;
            if (d_rx_ready_cnt1 % 10 == 0 && d_rxtime_offset > 0) // && d_clk_offset_ok
                send_rxtime();

            if (d_rx_ready_cnt1 > 20 && d_clk_offset_ok && d_rxtime_offset > 0
                && (!d_p2rx || d_rx_ready_cnt2 > 20) &&  (!d_p3rx || d_rx_ready_cnt3 > 20))
            {
                d_rx_ready = true;
                send_rxstate(true);
            }
            else if (d_rx_ready_cnt1 == 0 || (d_p2rx && d_rx_ready_cnt2 == 0) || (d_p3rx && d_rx_ready_cnt3 == 0) )
            {
                d_rx_ready = false;
                send_rxstate(false);
            }

            d_rx_ready_cnt1 += 1;

            break;
        }
        case P1DEFRAME:
        {
            // send tag command and check current timestamp
            if (d_rxtime_offset <= 0)
                send_tagcmd();

            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len1)
            {
                noutput_samples = d_frame_len1 - d_data_samples;
                if (!d_p2rx && !d_p3rx)  // only phase 1 reception
                {
                    d_wait_interval = d_next_p1frame_start - uint64_t(noutput_samples + d_ht_len1 + 5 * SYM_LEN) - nitems_read(0);
                }
                else if (d_p2rx)  // phase 1 & 2 reception
                {
                    d_wait_interval = d_next_p2frame_start - uint64_t(noutput_samples + d_ht_len2 + 5 * SYM_LEN) - nitems_read(0);
                    if (d_rx_ready_cnt2 == 0) // additional samples for sync search buffer
                        d_wait_interval -= d_max_corr_len / 4;
                }
                else  // phase 1 & 3 reception
                {
                    d_wait_interval = d_next_p3frame_start - uint64_t(noutput_samples + d_ht_len3 + 5 * SYM_LEN) - nitems_read(0);
                    if (d_rx_ready_cnt3 == 0) // additional samples for sync search buffer
                        d_wait_interval -= d_max_corr_len / 4;
                }
                d_wait_count = 0;
                d_state = WAIT1;
            }

            // skip initial frames with unstable FOE
            // wait for all other receiver all synced
            if (!d_p1rx || d_rx_ready_cnt1 <= 20)
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
                                 pmt::from_long(d_frame_len1),
                                 _id);
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

            if (d_rxtime_offset <= 0)
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
            int new_data_count = d_wait_count - d_wait_interval;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                if (!d_p2rx && !d_p3rx)
                    d_state = P1FINESYNC;
                else if (d_p2rx)
                    d_state = (d_rx_ready_cnt2 == 0) ? P2SEARCH : P2FINESYNC;
                else
                    d_state = (d_rx_ready_cnt3 == 0) ? P3SEARCH : P3FINESYNC;
            }

            if (d_rxtime_offset <= 0)
                check_rxtime(noutput_samples);
            consume_each(min_input_items);
            break;
        }
        case P2SEARCH:
        {
            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp2);
            if (start_pos < 0)
            {
                // frame start not found, discard samples
                consume_each(buffer_len - 2 * STF_LEN);
                d_skip_p2_frame = true;
                d_state = P2DEFRAME;
                break;
            }
            auto packet_start = nitems_read(0) + (uint64_t) start_pos;
            dout << "P2 packet start detected at " << packet_start << std::endl;
            consume_each(start_pos);
            d_sync_err_cnt2 = 0;
            d_skip_p2_frame = false;
            d_state = P2FINESYNC;
            break;
        }
        case P2FINESYNC:
        {
            if (min_input_items < 3 * LTF_LEN)
                break;

            // fine_sync return the start of the first HT preambles
            int ht_start = fine_sync(input_items, 3 * LTF_LEN, false,
                                     d_current_foe_comp2, d_fine_foe_comp2,
                                     d_rx_ready_cnt2, d_fine_foe_cnt2);

            uint64_t cur_frame_start = 0;
            if (ht_start < 0)
            {
                dout << "P2 fine synchronize failed!" << std::endl;
                // consume_each(min_input_items);
                d_sync_err_cnt2 += 1;
                if (d_sync_err_cnt2 >= 5)
                {
                    // changing from ready state
                    // std::cout << "======== P2 Receiver synchronization lost ========" << std::endl;
                    d_sync_err_cnt2 = 0;
                    d_rx_ready_cnt2 = 0;
                    d_prev_p2frame_start = 0;
                    d_skip_p2_frame = true;
                    // skip the whole beacon
                    consume_each(5 * SYM_LEN + d_ht_len2);
                    d_state = P2DEFRAME;
                    break;
                }
            }
            else
            {
                // skip HT preambles excluding LTFs
                ht_start += d_ht_len2;
                cur_frame_start = nitems_read(0) + ht_start;
                dout << "P2 data frame start at " << ht_start << " (" << cur_frame_start << ")" << std::endl;
            }

            if (d_rx_ready_cnt2 > 20 && (d_rx_ready_cnt2 % 100) == 0)
            {
                std::cout << "P2 fine frequency offset compensation: " << d_current_foe_comp2 << "    ("
                          << d_current_foe_comp2 * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            int64_t frame_offset = int64_t(cur_frame_start - d_prev_p2frame_start) - d_frame_interval;
            if (ht_start <= 0 || (d_prev_p2frame_start > 0 && (frame_offset < -SYM_LEN || frame_offset > SYM_LEN)))
            {
                // use frame start prediction
                cur_frame_start = d_prev_p2frame_start + uint64_t(d_frame_interval);
                ht_start = std::max(0, int(cur_frame_start - nitems_read(0)));
                std::cout << "P2 fine frame synchronization not correct, using prediction instead!" << std::endl;
            }
            d_next_p2frame_start = cur_frame_start + d_frame_interval;
            d_prev_p2frame_start = cur_frame_start;

            d_data_samples = 0;
            d_state = P2DEFRAME;
            d_rx_ready_cnt2 += 1;

            consume_each(ht_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            break;
        }
        case P2DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len2)
            {
                noutput_samples = d_frame_len2 - d_data_samples;
                if (!d_p3rx)
                    d_wait_interval = d_next_p1frame_start - uint64_t(noutput_samples + d_ht_len1 + 5 * SYM_LEN);
                else
                {
                    d_wait_interval = d_next_p3frame_start - uint64_t(noutput_samples + d_ht_len3 + 5 * SYM_LEN);
                    if (d_rx_ready_cnt3 == 0) // additional samples for sync search buffer
                        d_wait_interval -= d_max_corr_len / 4;
                }
                d_wait_interval -= nitems_read(0);
                d_wait_count = 0;
                d_state = WAIT2;
            }

            if (d_skip_p2_frame || !d_rx_ready)
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
                                 pmt::from_long(d_frame_len2),
                                 _id);
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
            int new_data_count = d_wait_count - d_wait_interval;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                if (!d_p3rx)
                    d_state = P1FINESYNC;
                else
                    d_state = (d_rx_ready_cnt3 == 0) ? P3SEARCH : P3FINESYNC;

            }

            if (d_rxtime_offset <= 0)
                check_rxtime(noutput_samples);
            consume_each(min_input_items);
            break;
        }
        case P3SEARCH:
        {
            // search for frame start
            int buffer_len = ((min_input_items > d_max_corr_len) ? d_max_corr_len : min_input_items);
            if (buffer_len < 8 * STF_LEN)
                break;
            int start_pos = sync_search(input_items, buffer_len, d_current_foe_comp3);
            if (start_pos < 0)
            {
                // frame start not found, discard samples
                consume_each(buffer_len - 2 * STF_LEN);
                d_skip_p3_frame = true;
                d_state = P3DEFRAME;
                break;
            }
            auto packet_start = nitems_read(0) + (uint64_t) start_pos;
            dout << "P3 packet start detected at " << packet_start << std::endl;
            consume_each(start_pos);
            d_sync_err_cnt3 = 0;
            d_skip_p3_frame = false;
            d_state = P3FINESYNC;
            break;
        }
        case P3FINESYNC:
        {
            if (min_input_items < 3 * LTF_LEN)
                break;

            // fine_sync return the start of the first HT preambles
            int ht_start = fine_sync(input_items, 3 * LTF_LEN, false,
                                     d_current_foe_comp3, d_fine_foe_comp3,
                                     d_rx_ready_cnt3, d_fine_foe_cnt3);

            uint64_t cur_frame_start = 0;
            if (ht_start < 0)
            {
                dout << "P3 fine synchronize failed!" << std::endl;
                // consume_each(min_input_items);
                d_sync_err_cnt3 += 1;
                if (d_sync_err_cnt3 >= 5)
                {
                    // changing from ready state
                    std::cout << "======== P3 Receiver synchronization lost ========" << std::endl;
                    d_sync_err_cnt3 = 0;
                    d_rx_ready_cnt3 = 0;
                    d_prev_p3frame_start = 0;
                    d_skip_p3_frame = true;
                    // skip the whole beacon
                    consume_each(5 * SYM_LEN + d_ht_len3);
                    d_state = P3DEFRAME;
                    break;
                }
            }
            else
            {
                // skip HT preambles excluding LTFs
                ht_start += d_ht_len3;
                cur_frame_start = nitems_read(0) + ht_start;
                dout << "P3 data frame start at " << ht_start << " (" << cur_frame_start << ")" << std::endl;
            }

            if (d_rx_ready_cnt3 > 20 && (d_rx_ready_cnt3 % 100) == 0)
            {
                std::cout << "P3 fine frequency offset compensation: " << d_current_foe_comp3 << "    ("
                          << d_current_foe_comp3 * d_samplerate / (2.0 * M_PI) << " Hz)" << std::endl;
            }

            int64_t frame_offset = int64_t(cur_frame_start - d_prev_p3frame_start) - d_frame_interval;
            if (ht_start <= 0 || (d_prev_p3frame_start > 0 && (frame_offset < -SYM_LEN || frame_offset > SYM_LEN)))
            {
                // use frame start prediction
                cur_frame_start = d_prev_p3frame_start + uint64_t(d_frame_interval);
                ht_start = std::max(0, int(cur_frame_start - nitems_read(0)));
                std::cout << "P3 fine frame synchronization not correct, using prediction instead!" << std::endl;
            }
            d_next_p3frame_start = cur_frame_start + d_frame_interval;
            d_prev_p3frame_start = cur_frame_start;

            d_data_samples = 0;
            d_state = P3DEFRAME;
            d_rx_ready_cnt3 += 1;

            consume_each(ht_start); // remove L-STF/L-LTF/L-SIG/HT-SIG/HT-STF
            break;
        }
        case P3DEFRAME:
        {
            noutput_samples = std::min(min_input_items, noutput_items);
            if (d_data_samples + noutput_samples >= d_frame_len3)
            {
                noutput_samples = d_frame_len3 - d_data_samples;
                d_wait_interval = d_next_p1frame_start - uint64_t(noutput_samples + d_ht_len1 + 5 * SYM_LEN) - nitems_read(0);
                d_wait_count = 0;
                d_state = WAIT3;
            }

            if (d_skip_p3_frame || !d_rx_ready)
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
                    add_item_tag(2 * d_num_chans + ch,
                                 nitems_written(2 * d_num_chans + ch),
                                 pmt::string_to_symbol("frame_start"),
                                 pmt::from_long(d_frame_len2),
                                 _id);
            }

            for (int i = 0; i < noutput_samples; i++)
            {
                gr_complex comp_val = exp(gr_complex(0, d_current_foe_comp3 * (double) d_data_samples));
                for (int ch = 0; ch < d_num_chans; ch++)
                {
                    auto in = (const gr_complex *) input_items[ch];
                    auto out = (gr_complex *) output_items[2 * d_num_chans + ch];
                    out[i] = in[i] * comp_val;
                }
                d_data_samples += 1;
            }

            consume_each(noutput_samples);
            for (int ch = 0; ch < d_num_chans; ch++)
            {
                // produce(ch, 0);
                produce(2 * d_num_chans + ch, noutput_samples);
            }
            break;
        }
        case WAIT3:
        {
            d_wait_count += min_input_items;
            int new_data_count = d_wait_count - d_wait_interval;
            if (new_data_count >= 0)
            {
                d_wait_count = 0;
                min_input_items -= new_data_count;
                d_state = P1FINESYNC;
            }

            consume_each(min_input_items);
            break;
        }
    }

    return WORK_CALLED_PRODUCE;
}

void
rx_sync_impl::send_tagcmd()
{
    pmt::pmt_t cmdmsg = pmt::make_dict();
    cmdmsg = pmt::dict_add(cmdmsg, pmt::mp("tag"), pmt::mp(""));
    message_port_pub(pmt::mp("uhdcmd"), cmdmsg);
}

void
rx_sync_impl::send_rxstate(bool ready)
{
    message_port_pub(pmt::mp("rxrdy"), pmt::from_bool(ready));
}

void
rx_sync_impl::check_rxtime(int rx_windows_size)
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

void
rx_sync_impl::send_rxtime()
{
    // adjusted frame start position
    uint64_t frame_start_offset = d_prev_p1frame_start + d_rxtime_offset;
    // double frame_start_time = (frame_start_offset / d_samplerate);

    // send current frame start position and clock offset estimate
    auto rxtime_msg = pmt::make_tuple(pmt::from_uint64(frame_start_offset),
                                      pmt::from_double(d_clk_offset_est));
    message_port_pub(pmt::mp("rxtime"), rxtime_msg);
}

/*
 * search for frame start using L-STF auto-corr peaks
 */
int
rx_sync_impl::sync_search(const gr_vector_const_void_star &input_items, int buffer_len, float &current_foe_comp)
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
            dout << "Auto-correlation peak found at " << peak_start
                 << " (peak duration " << peak_duration << ")" << std::endl;
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
rx_sync_impl::fine_sync(const gr_vector_const_void_star &input_items, int buffer_len, bool lltf2,
                        float &current_foe_comp, float &fine_foe_comp, uint64_t &rx_ready_cnt, int &fine_foe_cnt)
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
    if (lltf2)
        for (int ch = 0; ch < d_num_chans; ch++)
            d_xcorr_fir_2.filterN(&d_xcorr_buf[ch * MAX_XCORR_LEN], &d_input_buf[ch * XCORR_DATA_LEN], xcorr_len);
    else
        for (int ch = 0; ch < d_num_chans; ch++)
            d_xcorr_fir_1.filterN(&d_xcorr_buf[ch * MAX_XCORR_LEN], &d_input_buf[ch * XCORR_DATA_LEN], xcorr_len);

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
    if (first_peak_pos > 0 && second_peak_pos >= first_peak_pos + FFT_LEN - 2 * deltaCSD &&
        second_peak_pos <= first_peak_pos + FFT_LEN + 2 * deltaCSD)
    {
        // calculate the start of the HT preambles (HT-SIGs)
        // first_peak_pos corresponding to the start of the first FFT block in L-LTF
        ht_start = first_peak_pos + 2 * FFT_LEN + SYM_LEN; // - deltaCSD;

        // calculate the start of the HT-LTF (removing HT-SIG and HT-STF)
        // first_peak_pos corresponding to the start of the first FFT block in L-LTF
        // 4 * SYM_LEN : L-SIG, HT-SIG, HT-STF
        // ht_start = first_peak_pos + 2 * FFT_LEN + 4 * SYM_LEN;
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

    // return start position of HT preambles
    return ht_start;
}

// conjugated and reversed LTF sequence
const
std::vector<gr_complex> rx_sync_impl::LTF_SEQ_1 = {
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

// conjugated and reversed LTF sequence
const
std::vector<gr_complex> rx_sync_impl::LTF_SEQ_2 = {
    gr_complex(0.387815, -1.383373), gr_complex(0.629355, 0.030737), gr_complex(0.461168, 0.522862),
    gr_complex(-0.404599, 1.099085), gr_complex(0.961322, 0.288129), gr_complex(0.848315, -0.433988),
    gr_complex(-0.044150, 0.901229), gr_complex(-0.669582, 0.310998), gr_complex(-0.332859, 0.312715),
    gr_complex(1.009401, 0.210974), gr_complex(0.687483, -0.823841), gr_complex(-0.192325, 0.652262),
    gr_complex(-0.480830, 0.015011), gr_complex(0.018589, -1.452701), gr_complex(-0.661930, -0.736988),
    gr_complex(-0.832050, -0.832050), gr_complex(0.830611, 0.139572), gr_complex(-0.280670, 0.339226),
    gr_complex(-1.193270, -0.538236), gr_complex(-0.917076, 0.584965), gr_complex(-0.414197, 0.176768),
    gr_complex(0.962484, -0.255799), gr_complex(-0.139442, 1.321481), gr_complex(0.114882, 0.865699),
    gr_complex(0.876366, -0.323971), gr_complex(-0.926335, 0.208639), gr_complex(-0.994119, -0.096130),
    gr_complex(-0.704801, -1.187012), gr_complex(-0.596485, 0.153677), gr_complex(-0.042337, 0.713263),
    gr_complex(0.652517, -1.171849), gr_complex(1.386750, 0.000000), gr_complex(0.652517, 1.171849),
    gr_complex(-0.042337, -0.713263), gr_complex(-0.596485, -0.153677), gr_complex(-0.704801, 1.187012),
    gr_complex(-0.994119, 0.096130), gr_complex(-0.926335, -0.208639), gr_complex(0.876366, 0.323971),
    gr_complex(0.114882, -0.865699), gr_complex(-0.139442, -1.321481), gr_complex(0.962484, 0.255799),
    gr_complex(-0.414197, -0.176768), gr_complex(-0.917076, -0.584965), gr_complex(-1.193270, 0.538236),
    gr_complex(-0.280670, -0.339226), gr_complex(0.830611, -0.139572), gr_complex(-0.832050, 0.832050),
    gr_complex(-0.661930, 0.736988), gr_complex(0.018589, 1.452701), gr_complex(-0.480830, -0.015011),
    gr_complex(-0.192325, -0.652262), gr_complex(0.687483, 0.823841), gr_complex(1.009401, -0.210974),
    gr_complex(-0.332859, -0.312715), gr_complex(-0.669582, -0.310998), gr_complex(-0.044150, -0.901229),
    gr_complex(0.848315, 0.433988), gr_complex(0.961322, -0.288129), gr_complex(-0.404599, -1.099085),
    gr_complex(0.461168, -0.522862), gr_complex(0.629355, -0.030737), gr_complex(0.387815, 1.383373),
    gr_complex(1.386750, 0.000000),
};

} /* namespace gr::ncjt */
