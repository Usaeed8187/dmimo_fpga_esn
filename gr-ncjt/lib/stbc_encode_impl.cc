/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "stbc_encode_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"
#include "cmatrix.h"

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

    d_tracking_pilots = malloc_complex(16 * d_numsyms);
    generate_tracking_pilots();
}

stbc_encode_impl::~stbc_encode_impl()
{
    if (d_tracking_pilots != nullptr)
        volk_free(d_tracking_pilots);
}

int
stbc_encode_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_scnum * d_numsyms;
    return noutput_items;
}

void
stbc_encode_impl::generate_tracking_pilots()
{
    // LSFR state for pilot parity sequence
    unsigned d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x70; // offset = 3 or 4
    int d_npt = (d_fftsize == 64) ? 4 : 8;

    // base pilot sequence for 64-FFT & 2/4-stream case
    const float basePilots4[6][8] = {{1, 1, -1, -1}, {1, -1, -1, 1}, // 2Rx
                                     {1, 1, 1, -1}, {1, 1, -1, 1},  // 4rx
                                     {1, -1, 1, 1}, {-1, 1, 1, 1}}; // 4Rx
    // base pilot sequence for 256-FFT single-stream case
    const float basePilots8[4][8] = {{1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1}};

    auto basePilot = (d_fftsize == 64) ? basePilots4 : basePilots8;

    for (int symidx = 0; symidx < d_numsyms; symidx++)
    {
        // update pilot parity for current symbol
        unsigned parity_bit = (d_pilot_lsfr >> 6) ^ ((d_pilot_lsfr >> 3) & 0x1);
        d_pilot_lsfr = ((d_pilot_lsfr << 1) & 0x7E) | parity_bit; // circular 7-bit shifter
        float pilot_parity = (parity_bit == 1) ? -1 : 1;

        // generate current pilots
        for (int i = 0; i < d_npt; i++) // for all pilot positions
        {
            unsigned int idx = (symidx + i) % d_npt;
            for (int k = 0; k < 2; k++) // for all streams
            {
                int offset = symidx * d_npt * 2 + d_npt * k + i;
                d_tracking_pilots[offset] = gr_complex(float(pilot_parity * basePilot[k][idx]), 0.0);
            }
        }
    }
}

int
stbc_encode_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    // Reshape input as a tensor of shape (Nsc, 2, Nsyms/2)
    Eigen::DSizes<Eigen::Index, 3> dims(d_scnum, 2, d_numsyms / 2);
    CTensor3D xin = Eigen::TensorMap<const CTensor3D>(in, dims);

    // Extract odd/even-index symbols
    CTensor2D s0 = xin.chip(0, 1);  // (Nsc, Nsyms/2)
    CTensor2D s1 = xin.chip(1, 1);  // (Nsc, Nsyms/2)

    // Map output buffer as tensors of shape (Nsc, 2, Nsyms/2)
    auto tx0 = Eigen::TensorMap<CTensor3D>(out0, dims);
    auto tx1 = Eigen::TensorMap<CTensor3D>(out1, dims);

    // Tx antenna 1: (s0, -s1*)
    tx0.chip(0, 1) = s0;
    tx0.chip(1, 1) = -s1.conjugate();

    // Tx antenna 2: (s1, s0*)
    tx1.chip(0, 1) = s1;
    tx1.chip(1, 1) = s0.conjugate();

    // Generate packing pilots
    for (int n = 0; n < d_numsyms; n++)
    {
        int offset = d_scnum * n;
        out0[offset + 7] = d_tracking_pilots[8 * n];
        out0[offset + 21] = d_tracking_pilots[8 * n + 1];
        out0[offset + 34] = d_tracking_pilots[8 * n + 2];
        out0[offset + 48] = d_tracking_pilots[8 * n + 3];
        out1[offset + 7] = d_tracking_pilots[8 * n + 4];
        out1[offset + 21] = d_tracking_pilots[8 * n + 5];
        out1[offset + 34] = d_tracking_pilots[8 * n + 6];
        out1[offset + 48] = d_tracking_pilots[8 * n + 7];
    }

    return d_scnum * d_numsyms;
}

} /* namespace gr::ncjt */
