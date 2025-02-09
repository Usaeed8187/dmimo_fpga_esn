/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "rx_mrc_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

rx_mrc::sptr
rx_mrc::make(int fft_size, int nrx, int nsymbols, bool debug)
{
    return gnuradio::make_block_sptr<rx_mrc_impl>(fft_size, nrx, nsymbols, debug);
}

rx_mrc_impl::rx_mrc_impl(int fft_size, int nrx, int nsymbols, bool debug)
    : gr::tagged_stream_block(
    "rx_mrc",
    gr::io_signature::make(nrx, nrx, sizeof(gr_complex)),
    gr::io_signature::make(2, nrx + 2, sizeof(gr_complex)),
    "packet_len"),
      d_debug(debug)
{
    if (nrx < 1 || nrx > 8)
        throw std::runtime_error("only sport 1 to 8 receivers");
    d_nrx = nrx;

    if (fft_size == 64)
    {
        d_scnum = 56;
        d_sdnum = 52;
    }
    else
    {
        d_scnum = 242;
        d_sdnum = 234;
    }

    if (nsymbols < 1 || nsymbols > 200)
        throw std::runtime_error("invalid number of OFDM data symbols");
    d_num_symbols = nsymbols;

    d_cur_symbol = -1;
    d_chan_est_ready = false;
    d_total_pkts = 0;
    d_chan_est = malloc_complex(d_scnum * d_nrx);
    for (int k = 0; k < 8; k++)
        d_chan_scaling[k] = 1.0;

    set_tag_propagation_policy(block::TPP_DONT);
}

rx_mrc_impl::~rx_mrc_impl()
{
    volk_free(d_chan_est);
}

int
rx_mrc_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nrx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int noutput_items = d_sdnum * (min_input_items / d_scnum);

    return noutput_items;
}

void
rx_mrc_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int ch = 0; ch < d_nrx + 2; ch++)
        add_item_tag(ch, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len),
                     pmt::string_to_symbol(name()));
}

int
rx_mrc_impl::work(int noutput_items, gr_vector_int &ninput_items,
                  gr_vector_const_void_star &input_items,
                  gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nrx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int ninput_syms = min_input_items / d_scnum;
    if (ninput_syms <= 0)
        return 0;

    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 1, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum - 1,
                       pmt::string_to_symbol("noise_var"));
    if (d_tags.empty())
    {
        dout << "ERROR: cannot find noise_var tag" << std::endl;
        return 0;
    }
    float nvar = pmt::to_float(d_tags[0].value);
    float nvar_scaling = 1.0f / nvar;

    // MRC combining and equalization
    auto eqdata = (gr_complex *) output_items[0];
    auto csidata = (gr_complex *) output_items[1];
    int output_syms = 0;
    for (int k = 0; k < ninput_syms; k++)
    {
        if (d_cur_symbol == -1)
        {
            std::vector<gr::tag_t> pkt_tags;
            get_tags_in_window(pkt_tags, 0, 0, 1, pmt::string_to_symbol("packet_start"));
            if (!pkt_tags.empty())  //  packet start detected
            {
                for (int ch = 0; ch < d_nrx; ch++)
                {
                    // copy channel estimation conjugates
                    auto *in = (const gr_complex *) input_items[ch];
                    gr_complex *chan_est = &d_chan_est[ch * d_scnum];
                    float power_est = 0.0;
                    for (int i = 0; i < d_scnum; i++)
                    {
                        chan_est[i] = conj(in[i]);  // conjugate channel
                        power_est += std::norm(in[i]);
                    }
                    power_est /= (float) d_scnum;
                    d_chan_scaling[ch] = 1.0f / std::sqrt(power_est);
                }
                d_cur_symbol += 1;
                d_chan_est_ready = true;
                d_total_pkts += 1;
                dout << "Total number of packets received: " << d_total_pkts << std::endl;
                continue;
            }
        }

        if (!d_chan_est_ready)
            throw std::runtime_error("channel estimation not received before demapping");

        // MRC processing
        for (int i = 0, sc_cnt = 0; i < d_scnum; i++)
        {
            if (i == 7 || i == 21 || i == 34 || i == 48)
                continue;

            gr_complex ys = 0;
            float hs = 0;
            for (int ch = 0; ch < d_nrx; ch++)
            {
                auto *in = (const gr_complex *) input_items[ch];
                ys += d_chan_scaling[ch] * in[k * d_scnum + i] * d_chan_est[ch * d_scnum + i];
                hs += d_chan_scaling[ch] * norm(d_chan_est[ch * d_scnum + i]);
            }
            unsigned oaddr = output_syms * d_sdnum + sc_cnt;
            eqdata[oaddr] = ys;  // equalized QAM data stream
            csidata[oaddr] = gr_complex(hs, nvar_scaling); // CSI and noise scaling

            sc_cnt += 1;
        }

        // Individual Rx processing
        for (int ch = 0; ch < d_nrx; ch++)
        {
            auto *ry = (const gr_complex *) input_items[ch];
            auto *qamconst = (gr_complex *) output_items[2 + ch];
            int sc_cnt = 0;
            for (int i = 0; i < d_scnum; i++)
            {
                if (i == 7 || i == 21 || i == 34 || i == 48)
                    continue;
                unsigned oaddr = output_syms * d_sdnum + sc_cnt;
                qamconst[oaddr] = ry[k * d_scnum + i] / conj(d_chan_est[ch * d_scnum + i]);
                sc_cnt += 1;
            }
        }

        output_syms += 1;
        d_cur_symbol += 1;
        if (d_cur_symbol == d_num_symbols)
        {
            d_cur_symbol = -1;
            d_chan_est_ready = false;
        }
    }

    if (output_syms > 0)
    {
        uint64_t offset = nitems_written(0);
        add_packet_tag(offset, d_sdnum * output_syms);
    }

    return output_syms * d_sdnum;
}

} /* namespace gr::ncjt */
