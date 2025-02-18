/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "mc_stbc_decode_impl.h"
#include <gnuradio/io_signature.h>

#include "qam_modulation.h"
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
                    // row major memory layout for Eigen
                    int offset = 2 * d_scnum * m + d_scnum * n + k;
                    d_chan_est[offset] = in[2 * k + m];
                }
            }
    }
    else
        throw std::runtime_error("Channel estimation not received!");

    // reshape inputs as a tensor of shape (Nsyms/2, 2, Nsc)
    Eigen::DSizes<Eigen::Index, 4> dims(1, d_numsyms / 2, 2, d_scnum);
    CTensor4D ry0 = Eigen::TensorMap<const CTensor4D>(in0 + 2 * d_scnum, dims);
    CTensor4D ry1 = Eigen::TensorMap<const CTensor4D>(in1 + 2 * d_scnum, dims);
    CTensor4D ry = ry0.concatenate(ry1, 0);
    dout << "ry size: " << ry.dimensions() << std::endl;

    // channel estimation
    Eigen::DSizes<Eigen::Index, 4> chest_dims(2, 2, 1, d_scnum);
    auto chant_est = Eigen::TensorMap<const CTensor4D>(d_chan_est, chest_dims);
    Eigen::array<int, 4> bcast({1, 1, d_numsyms / 2, 1});
    CTensor4D ch = chant_est.broadcast(bcast);
    dout << "chanest size: " << ch.dimensions() << std::endl;

    // STBC receiver
    auto [z, h_eq] = alamouti_decode(ry, ch);
    dout << "output size: " << z.dimensions() << std::endl;

    // symbols & csi output
    for (int m = 0; m < d_numsyms; m++)
    {
        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;
            int offset = m * d_scdata + sc_cnt;
            out0[offset] = z(m, i);
            out1[offset] = gr_complex(h_eq(m, i), 0.0);
            sc_cnt += 1;
        }
    }

    return (d_scdata * d_numsyms);
}

/* Decodes symbols received from M_r receive antennas using Alamouti decoding.
 *
 * @param r Received symbols of shape [num_rx, num_syms_half, 2, num_subcarriers]
 * @param h Channel estimation of shape [num_rx, 2, num_syms_half, num_subcarriers]
 *
 * @return A tuple of two tensors:
 *   z: Estimated symbols of shape [num_syms, num_subcarriers]
 *   h_eq: Effective channel gain of shape [num_syms, num_subcarriers]
 */
std::tuple<CTensor2D, Tensor2D>
mc_stbc_decode_impl::alamouti_decode(const CTensor4D &r, const CTensor4D &h)
{
    // Split r into r1 and r2
    CTensor3D r1 = r.chip(0, 2); // Slice [num_rx, num_syms/2, num_subcarriers]
    CTensor3D r2 = r.chip(1, 2);

    // Split h into h1 and h2
    CTensor3D h1 = h.chip(0, 0); // Slice [num_rx, num_syms/2, num_subcarriers]
    CTensor3D h2 = h.chip(1, 0);

    // z1 = h1^* * r1 + h2 * r2^*
    CTensor3D z1 = h1.conjugate() * r1 + h2 * r2.conjugate(); // [num_rx, num_syms/2, num_subcarriers]

    // z2 = h2^* * r1 - h1 * r2^*
    CTensor3D z2 = h2.conjugate() * r1 - h1 * r2.conjugate(); // [num_rx, num_syms/2, num_subcarriers]

    // Step 1: Reshape z1 and z2 to [num_rx, num_syms_half, 1, num_subcarriers]
    Eigen::array<int, 4> z_reshaped_dims = {d_nrx, d_numsyms / 2, 1, d_scnum};
    CTensor4D z1_reshaped = Eigen::TensorMap<CTensor4D>(z1.data(), z_reshaped_dims);
    CTensor4D z2_reshaped = Eigen::TensorMap<CTensor4D>(z2.data(), z_reshaped_dims);

    // Step 2: Concatenate along axis 1 to produce [num_rx, num_syms_half, 2, num_subcarriers]
    CTensor4D z_combined = z1_reshaped.concatenate(z2_reshaped, 2);

    // Step 3: Reshape concatenated tensor to [num_rx, 2 * num_syms_half, num_subcarriers]
    Eigen::array<int, 3> z_combined_dims = {d_nrx, d_numsyms, d_scnum};
    CTensor3D z_combined_reshaped = Eigen::TensorMap<CTensor3D>(z_combined.data(), z_combined_dims);
    dout << "z_combined_reshaped: " << z_combined_reshaped.dimensions() << std::endl;

    // Step 4: Sum over receive antennas (axis 0) to get [2 * num_syms_half, num_subcarriers]
    CTensor2D z_summed = z_combined_reshaped.sum(Eigen::array<int, 1>{0});

    // Effective channel gain
    Tensor3D h_eq = h1.abs().pow(2) + h2.abs().pow(2); // [M_r, num_syms/2, num_subcarriers]
    Tensor2D h_eq_summed = h_eq.sum(Eigen::array<int, 1>{0}); // [num_syms/2, num_subcarriers]

    // Step 1: Reshape h_eq_summed to [1, num_syms/2, num_subcarriers]
    Eigen::array<int, 3> h_eq_intermediate_dims = {1, d_numsyms / 2, d_scnum};
    Tensor3D h_eq_intermediate = Eigen::TensorMap<Tensor3D>(h_eq_summed.data(), h_eq_intermediate_dims);

    // Step 2: Concatenate with itself to form [2, num_syms/2, num_subcarriers]
    Tensor3D h_eq_duplicated = h_eq_intermediate.concatenate(h_eq_intermediate, 0);

    // Step 3: Reshape to final dimensions [num_syms, num_subcarriers]
    Eigen::array<int, 2> h_eq_final_dims = {d_numsyms, d_scnum};
    Tensor2D h_eq_reshaped = Eigen::TensorMap<Tensor2D>(h_eq_duplicated.data(), h_eq_final_dims);

    return {z_summed, h_eq_reshaped};
}

} /* namespace gr::ncjt */
