/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ofdm_demod_impl.h"
#include <gnuradio/io_signature.h>

namespace gr::ncjt
{

ofdm_demod::sptr
ofdm_demod::make(int fftsize, int cplen, int symoffset)
{
    return gnuradio::make_block_sptr<ofdm_demod_impl>(fftsize, cplen, symoffset);
}

ofdm_demod_impl::ofdm_demod_impl(int fftsize, int cplen, int symoffset)
    : gr::tagged_stream_block(
    "ofdm_demod",
    gr::io_signature::make(1, 1, sizeof(gr_complex)),
    gr::io_signature::make(1, 1, sizeof(gr_complex)),
    "frame_start"),
      d_fft(fftsize, 1)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;

    if (cplen <= 0 || cplen > fftsize / 4 || (cplen != 16 && cplen != 32 && cplen != 64))
        throw std::runtime_error("Unsupported OFDM CP length");
    d_cplen = cplen;
    d_symlen = d_fftsize + d_cplen;

    if (d_fftsize == 64)
    {
        d_scnum = 56;
        d_scnum_half = 28;
        d_left_guard_scnum = 4;
        d_center_null_scnum_pos_half = 1;
    }
    else
    {
        d_scnum = 242;
        d_scnum_half = 121;
        d_left_guard_scnum = 6;
        d_center_null_scnum_pos_half = 2;
    }

    if (symoffset > 0 && symoffset < d_cplen)
        d_symbol_offset = symoffset;
    else
        d_symbol_offset = 12; // default value

    set_tag_propagation_policy(block::TPP_DONT);
}

ofdm_demod_impl::~ofdm_demod_impl()
{
}

int
ofdm_demod_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int nblks = ninput_items[0] / (d_symlen);
    return (nblks * d_scnum);
}

int
ofdm_demod_impl::work(int noutput_items, gr_vector_int &ninput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    const gr_complex *in0 = (const gr_complex *) input_items[0];
    gr_complex *out0 = (gr_complex *) output_items[0];

    int num_blks = std::min(ninput_items[0] / d_symlen, noutput_items / d_scnum);
    if (num_blks <= 0)
        return 0;

    int nblk = 0; // total number of output blocks
    for (int i = 0; i < num_blks; i++)
    {
        // start position for current OFDM symbol
        unsigned int input_offset = i * d_symlen + d_symbol_offset;

        // remove CP
        memcpy(d_fft.get_inbuf(), &in0[input_offset], sizeof(gr_complex) * d_fftsize);
        // compute FFT
        d_fft.execute();
        // FFT shift
        memcpy(&out0[nblk * d_scnum],
               &d_fft.get_outbuf()[d_fftsize / 2 + d_left_guard_scnum],
               sizeof(gr_complex) * d_scnum_half);
        memcpy(&out0[nblk * d_scnum + d_scnum_half],
               &d_fft.get_outbuf()[d_center_null_scnum_pos_half],
               sizeof(gr_complex) * d_scnum_half);

        nblk += 1;
    }

    add_item_tag(0, nitems_written(0),
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(nblk * d_scnum),
                 pmt::string_to_symbol(name()));

    return nblk * d_scnum;
}

} /* namespace gr::ncjt */
