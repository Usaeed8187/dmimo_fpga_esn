/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "mc_stbc_decode_impl.h"
#include <gnuradio/io_signature.h>

#include "stbc_sic_decode.h"
#include "utils.h"

namespace gr::ncjt
{

mc_stbc_decode::sptr
mc_stbc_decode::make(int fftsize, int ndatasyms, int npilotsyms, bool debug)
{
    return gnuradio::make_block_sptr<mc_stbc_decode_impl>(fftsize, ndatasyms, npilotsyms, debug);
}

mc_stbc_decode_impl::mc_stbc_decode_impl(int fftsize, int ndatasyms,
                                         int npilotsyms, bool debug)
    : gr::tagged_stream_block(
    "stbc_decode",
    gr::io_signature::make(2, 2, sizeof(gr_complex)),
    gr::io_signature::make(1, 2, sizeof(gr_complex)),
    "packet_len"), d_debug(debug)
{
    if (fftsize == 64)
    {
        d_scnum = 56;
        d_scdata = 52;
    }
    else if (fftsize == 256)
    {
        d_scnum = 242;
        d_scdata = 234;
    }
    else
        throw std::runtime_error("Unsupported OFDM FFT size");

    if (ndatasyms <= 0 || ndatasyms % 2 != 0 || npilotsyms < 0 || npilotsyms > 2)
        throw std::runtime_error("Invalid number of data/pilot OFDM symbols");

    d_ndatasyms = ndatasyms;
    d_npilotsyms = npilotsyms;
    d_numsyms = ndatasyms + npilotsyms;

    d_chan_est = malloc_complex(4 * d_scnum);

    set_tag_propagation_policy(block::TPP_DONT);
}

mc_stbc_decode_impl::~mc_stbc_decode_impl()
{
    if (d_chan_est != nullptr)
        volk_free(d_chan_est);
}

int
mc_stbc_decode_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_scnum * d_numsyms;
    return noutput_items;
}

int
mc_stbc_decode_impl::work(int noutput_items, gr_vector_int &ninput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto in1 = (const gr_complex *) input_items[1];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, 1,
                       pmt::string_to_symbol("packet_start"));
    if (!d_tags.empty())
    {
        // save channel estimation
        for (int k = 0; k < d_scnum; k++)
            for (int n = 0; n < 2; n++) // for all rx
            {
                const gr_complex *in = (const gr_complex *) input_items[n];
                for (int m = 0; m < 2; m++) // for all tx antennas
                {
                    // matrix shape (num_tx, num_rx, num_sc)
                    // column major memory layout for Eigen
                    int offset = 2 * d_scnum * m + d_scnum * n + k;
                    d_chan_est[offset] = in[2 * k + m];
                }
            }
    }
    else
        throw std::runtime_error("Channel estimation not received!");

    // reshape inputs as a tensor of shape (Nsyms/2, 2, Nsc)
    Eigen::DSizes<Eigen::Index, 3> dims(1, d_numsyms, d_scnum);
    CTensor3D ry0 = Eigen::TensorMap<const CTensor3D>(in0 + 2 * d_scnum, dims);
    CTensor3D ry1 = Eigen::TensorMap<const CTensor3D>(in1 + 2 * d_scnum, dims);
    CTensor3D ry = ry0.concatenate(ry1, 0);
    dout << "ry size: " << ry.dimensions() << std::endl;

    // channel estimation
    Eigen::DSizes<Eigen::Index, 4> chest_dims(2, 2, 1, d_scnum);
    auto chant_est = Eigen::TensorMap<const CTensor4D>(d_chan_est, chest_dims);
    Eigen::array<int, 4> bcast({1, 1, d_numsyms / 2, 1});
    CTensor4D ch = chant_est.broadcast(bcast);
    dout << "chanest size: " << ch.dimensions() << std::endl;

    // STBC receiver
    auto [z, h_eq] = alamouti_decode_zf_sic_double(ry, ch, d_modtype);
    dout << "output size: " << z.dimensions() << std::endl;

    // symbols & csi output
    for (int s = 0; s < 2; s++)
    for (int m = 0; m < d_numsyms; m++)
    {
        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;
            int offset = m * d_scdata + sc_cnt;
            out0[offset] = z(s, m, i);
            out1[offset] = gr_complex(h_eq(s, m, i), 0.0);
            sc_cnt += 1;
        }
    }

    return (d_scdata * d_numsyms);
}


} /* namespace gr::ncjt */
