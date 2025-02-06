/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mimo_detect_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"
#include <Eigen/Dense>

namespace gr::ncjt
{

mimo_detect::sptr
mimo_detect::make(int fftsize, int nss, int ndatasymbols, int logfreq, bool debug)
{
    return gnuradio::make_block_sptr<mimo_detect_impl>(fftsize, nss, ndatasymbols, logfreq, debug);
}

mimo_detect_impl::mimo_detect_impl(int fftsize, int nss, int ndatasymbols, int logfreq, bool debug)
    : gr::tagged_stream_block("mimo_detect",
                              gr::io_signature::make(nss, nss, sizeof(gr_complex)),
                              gr::io_signature::make(nss, nss, sizeof(gr_complex)),
                              "packet_len"),
      d_num_symbols(ndatasymbols), d_logfreq(logfreq), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    if (fftsize == 64)
    {
        d_scnum = 56;
        d_scdata = 52;
    }
    else
    {
        d_scnum = 242;
        d_scdata = 234;
    }

    if (nss < 1 || nss > 8)
        throw std::runtime_error("only support 1 to 8 streams");
    // assuming nrx == nss
    d_nrx = nss, d_nss = nss;

    d_chan_est_buf = malloc_complex(d_nrx * d_nss * d_scnum);
    d_mmse_coef = malloc_complex(d_nrx * d_nss * d_scnum);

    d_chan_est_ready = false;
    d_total_pkts = 0;
    set_tag_propagation_policy(block::TPP_DONT);
}

mimo_detect_impl::~mimo_detect_impl()
{
    if (d_chan_est_buf != nullptr)
        volk_free(d_chan_est_buf);
    if (d_mmse_coef != nullptr)
        volk_free(d_mmse_coef);
}

int
mimo_detect_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int min_input_items = ninput_items[0];
    for (int k = 1; k < d_nrx; k++)
        min_input_items = std::min(min_input_items, ninput_items[k]);
    int noutput_items = d_scdata * (min_input_items / d_scnum);
    return noutput_items;
}

int
mimo_detect_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    int cur_sym = 0, out_sym = 0;
    int ninput_syms = std::min(noutput_items / d_scdata, std::min(ninput_items[0], ninput_items[1]) / d_scnum);
    if (ninput_syms <= 0)
        return 0;

    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, 1,
                       pmt::string_to_symbol("packet_start"));
    if (d_tags.size() > 0)
    {
        // save channel estimation (column major storage)
        for (int k = 0; k < d_scnum; k++)
            for (int m = 0; m < d_nss; m++) // for all streams
            {
                const gr_complex *in = (const gr_complex *) input_items[m];
                for (int n = 0; n < d_nrx; n++) // for all rx
                {
                    int offset = k * d_nrx * d_nrx + n * d_nrx + m;
                    d_chan_est_buf[offset] = in[k * d_nrx + n];
                }
            }
        d_chan_est_ready = true;
        d_cur_symbol = 0;
        cur_sym += d_nrx;
        d_total_pkts += 1;
        dout << "Total number of packets received: " << d_total_pkts << std::endl;
    }

    if (!d_chan_est_ready)
        throw std::runtime_error("Channel estimation not received before demapping!");

    get_tags_in_window(d_tags, 1, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum - 1,
                       pmt::string_to_symbol("noise_var"));
    if (d_tags.empty())
    {
        dout << "ERROR: cannot find noise_var tag" << std::endl;
        return 0;
    }
    float nvar = pmt::to_float(d_tags[0].value);

    // MMSE symbol detection and CSI
    if (d_nrx == 2)
        update_mmse_coef_2rx(nvar);
    else if (d_nrx == 4)
        update_mmse_coef_4rx(nvar);
    else
        update_mmse_coef_nrx(nvar);

    // equalization and demapping
    for (; cur_sym < ninput_syms; cur_sym++, out_sym++)
    {
        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;

            if ((d_scnum == 242)
                & (i == 6 || i == 32 || i == 74 || i == 100 || i == 141 || i == 167 || i == 209 || i == 235))
                continue;

            unsigned raddr = cur_sym * d_scnum + i;
            unsigned oaddr = out_sym * d_scdata + sc_cnt;
            unsigned caddr = d_nss * d_nrx * i;
            Eigen::MatrixXcf X(1, d_nrx), Y(1, d_nrx);
            Eigen::Map<Eigen::MatrixXcf> H(&d_mmse_coef[caddr], d_nrx, d_nrx);
            for (int m = 0; m < d_nrx; m++)
            {
                const gr_complex *in = (const gr_complex *) input_items[m];
                X(0, m) = in[raddr];
            }
            Y = X * H;
            for (int n = 0; n < d_nrx; n++)
            {
                gr_complex *out = (gr_complex *) output_items[n];
                out[oaddr] = Y(0, n);
            }
            sc_cnt += 1;
        }
        d_cur_symbol += 1;
        if (d_cur_symbol == d_num_symbols)
        {
            d_cur_symbol = 0;
            d_chan_est_ready = false;
        }
    }

    uint64_t offset = nitems_written(0);
    add_packet_tag(offset, d_scdata * out_sym);

    return (d_scdata * out_sym);
}

void
mimo_detect_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int ch = 0; ch < d_nrx; ch++)
        add_item_tag(ch, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len),
                     pmt::string_to_symbol(name()));
}

void
mimo_detect_impl::update_mmse_coef_2rx(float nvar)
{
    Eigen::Matrix2cf Hc, HH;
    Eigen::Matrix2cf sigma = 0.5 * nvar * Eigen::Matrix2cf::Identity(2, 2);

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nSTS x nRx)
        Eigen::Map<Eigen::Matrix2cf> H(&d_chan_est_buf[4 * k], 2, 2);
        Eigen::Map<Eigen::Matrix2cf> G(&d_mmse_coef[4 * k], 2, 2);
        // compute HH'+N
        Hc = H.conjugate();
        HH = H * Hc + sigma;
        // G = H' * inv(HH'+N)
        G = Hc * HH.inverse();
    }
}

void
mimo_detect_impl::update_mmse_coef_4rx(float nvar)
{
    Eigen::Matrix4cf Hc, HH;
    Eigen::Matrix4cf sigma = 0.5 * nvar * Eigen::Matrix4cf::Identity(4, 4);

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nSTS x nRx)
        Eigen::Map<Eigen::Matrix4cf> H(&d_chan_est_buf[16 * k], 4, 4);
        Eigen::Map<Eigen::Matrix4cf> G(&d_mmse_coef[16 * k], 4, 4);
        // compute HH'+N
        Hc = H.conjugate();
        HH = H * Hc + sigma;
        // G = H' * inv(HH'+N)
        G = Hc * HH.inverse();
    }
}

void
mimo_detect_impl::update_mmse_coef_nrx(float nvar)
{
    Eigen::MatrixXcf Ha(d_nrx, d_nss), HH(d_nss, d_nss);
    Eigen::MatrixXcf sigma = nvar * Eigen::MatrixXcf::Identity(d_nrx, d_nrx);

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nSTS x nRx)
        Eigen::Map<Eigen::MatrixXcf> H(&d_chan_est_buf[16 * k], d_nss, d_nrx);
        Eigen::Map<Eigen::Matrix4cf> G(&d_mmse_coef[16 * k], d_nss, d_nrx);
        // compute HH'+N
        Ha = H.adjoint();
        HH = Ha * H + sigma;
        // G = inv(HH'+N) * H'
        G = HH.inverse() * Ha;
    }
}

} /* namespace gr::ncjt */
