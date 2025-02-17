/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "stbc_encode_impl.h"
#include <gnuradio/io_signature.h>
#include "cmatrix.h"
#include "utils.h"

namespace gr::ncjt
{

stbc_encode::sptr
stbc_encode::make(int fftsize, int ndatasyms, int npilotsyms, bool debug)
{
    return gnuradio::make_block_sptr<stbc_encode_impl>(fftsize, ndatasyms, npilotsyms, debug);
}

stbc_encode_impl::stbc_encode_impl(int fftsize, int ndatasyms, int npilotsyms, bool debug)
    : gr::tagged_stream_block(
    "stbc_encode",
    gr::io_signature::make(1, 1, sizeof(gr_complex)),
    gr::io_signature::make(2, 2, sizeof(gr_complex)),
    "packet_len"), d_debug(debug)
{
    d_fftsize = fftsize;
    if (d_fftsize == 64)
        d_scnum = 56;
    else if (d_fftsize == 256)
        d_scnum = 242;
    else
        throw std::runtime_error("Unsupported OFDM FFT size");

    if (ndatasyms <= 0 || ndatasyms % 2 != 0 || npilotsyms < 0 || npilotsyms > 2)
        throw std::runtime_error("Invalid number of data/pilot OFDM symbols");

    d_ndatasyms = ndatasyms;
    d_npilotsyms = npilotsyms;
    d_numsyms = ndatasyms + npilotsyms;

}

stbc_encode_impl::~stbc_encode_impl()
{
}

int
stbc_encode_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_scnum * d_numsyms;
    return noutput_items;
}

int
stbc_encode_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    // Reshape input as a tensor of shape (Nsyms/2, 2, Nsc)
    Eigen::DSizes<Eigen::Index, 3> dims(d_numsyms / 2, 2, d_scnum);
    CTensor3D xin = Eigen::TensorMap<const CTensor3D>(in, dims);

    // Extract odd/even-index symbols
    CTensor2D s0 = xin.chip(0, 1);  // (Nsyms/2, Nsc)
    CTensor2D s1 = xin.chip(1, 1);  // (Nsyms/2, Nsc)

    // Map output buffer as tensors of shape (Nsyms/2, 2, Nsc)
    auto tx0 = Eigen::TensorMap<CTensor3D>(out0, dims);
    auto tx1 = Eigen::TensorMap<CTensor3D>(out1, dims);

    // Tx antenna 1: (s0, -s1*)
    tx0.chip(0, 1) = s0;
    tx0.chip(1, 1) = -s1.conjugate();

    // Tx antenna 2: (s1, s0*)
    tx1.chip(0, 1) = s1;
    tx1.chip(1, 1) = s0.conjugate();

    // restore the tracking pilots
    // scidx = {7, 21, 34, 48} for fftsize=64
    for (int n = 0; n < d_numsyms; n++)
    {
        int offset = d_scnum * n;
        out0[offset + 7] = in[offset + 7];
        out0[offset + 21] = in[offset + 21];
        out0[offset + 34] = in[offset + 34];
        out0[offset + 48] = in[offset + 48];
        out1[offset + 7] = in[offset + 7];
        out1[offset + 21] = in[offset + 21];
        out1[offset + 34] = in[offset + 34];
        out1[offset + 48] = in[offset + 48];
    }

    return d_scnum * d_numsyms;
}

} /* namespace gr::ncjt */
