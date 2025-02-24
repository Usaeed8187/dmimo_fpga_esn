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
mc_stbc_decode::make(int fftsize, int ndatasyms, int npilotsyms, int modtype, bool debug)
{
    return gnuradio::make_block_sptr<mc_stbc_decode_impl>(fftsize, ndatasyms, npilotsyms, modtype, debug);
}

mc_stbc_decode_impl::mc_stbc_decode_impl(int fftsize, int ndatasyms, int npilotsyms, int modtype, bool debug)
    : gr::tagged_stream_block(
    "stbc_decode",
    gr::io_signature::make(2, 2, sizeof(gr_complex)),
    gr::io_signature::make(2, 6, sizeof(gr_complex)),
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

    if (modtype != 2 && modtype != 4 && modtype != 6 && modtype != 8)
        throw std::runtime_error("Invalid modulation order");
    d_modtype = modtype;

    d_ndatasyms = ndatasyms;
    d_npilotsyms = npilotsyms;
    d_numsyms = ndatasyms + npilotsyms;

    d_chan_est = malloc_complex(4 * d_nrx * d_scnum);

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
    int noutput_items = d_scdata * d_numsyms;
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

    if (ninput_items[0] < d_scnum * (4 + d_numsyms) ||
        ninput_items[1] < d_scnum * (4 + d_numsyms))
    {
        dout << "Input data length: " << ninput_items[0] << std::endl;
        throw std::runtime_error("Input data length incorrect!");
    }

    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, 1,
                       pmt::string_to_symbol("packet_start"));
    if (!d_tags.empty())
    {
        // save channel estimation
        for (int k = 0; k < d_scnum; k++)
            for (int n = 0; n < d_nrx; n++) // for all rx
            {
                const gr_complex *in = (const gr_complex *) input_items[n];
                for (int m = 0; m < d_ntx; m++) // for all tx antennas
                {
                    // matrix shape (num_rx, num_tx, num_sc)
                    // column major memory layout for Eigen
                    int offset = d_nrx * d_ntx * k + d_nrx * m + n;
                    d_chan_est[offset] = in[d_ntx * k + m];
                }
            }
    }
    else
        throw std::runtime_error("Channel estimation not received!");

    // reshape inputs as a tensor of shape (Nsc, 1, Nsyms)
    Eigen::DSizes<Eigen::Index, 3> dims(d_scnum, 1, d_numsyms);
    CTensor3D ry0 = Eigen::TensorMap<const CTensor3D>(in0 + d_ntx * d_scnum, dims);
    CTensor3D ry1 = Eigen::TensorMap<const CTensor3D>(in1 + d_ntx * d_scnum, dims);
    CTensor3D ry = ry0.concatenate(ry1, 1); // (num_sc, num_rx, num_syms)
    Eigen::array<int, 3> ry_dims = {1, 2, 0}; // transpose to (num_rx, num_syms, num_sc)
    CTensor3D r = ry.shuffle(ry_dims);
    dout << "ry size: " << r.dimensions() << std::endl;

    // channel estimation
    Eigen::DSizes<Eigen::Index, 4> chest_dims(d_nrx, d_ntx, 1, d_scnum);
    auto chant_est = Eigen::TensorMap<const CTensor4D>(d_chan_est, chest_dims);
    Eigen::array<int, 4> bcast({1, 1, d_numsyms, 1});
    CTensor4D h = chant_est.broadcast(bcast);  // (num_rx, num_tx, num_syms, num_sc)
    dout << "ch size: " << h.dimensions() << std::endl;

    // double-cluster STBC receiver
    // auto [z, h_eq] = alamouti_decode_zf_sic_double(ry_transposed, ch, d_modtype);
    // dout << "output size: " << z.dimensions() << std::endl;

    int M_r = r.dimension(0);
    int num_syms = r.dimension(1);
    int num_subcarriers = r.dimension(2);
    int num_syms_half = num_syms / 2;

//    assert(M_r % 2 == 0 && "M_r must be even.");
//    assert(h.dimension(0) == M_r && "Mismatch between M_r of the received signal and channel.");
//    assert(h.dimension(1) == 4 && "N_t (2nd dimension of h) must be 4.");
//    assert(h.dimension(2) == num_syms && "Mismatch between num_syms of the received signal and channel.");
//    assert(h.dimension(3) == num_subcarriers && "Mismatch between num_subcarriers of the received signal and channel.");

    CTensor4D r_reshaped = Eigen::TensorMap<CTensor4D>(
        r.data(), Eigen::array<int, 4>{M_r, 2, num_syms_half, num_subcarriers});

    Eigen::array<int, 4> transpose_dims = {1, 0, 2, 3}; // Transpose to [N_t, M_r, num_syms, num_subcarriers]
    CTensor4D h_transposed = h.shuffle(transpose_dims);

    // Reshape to [N_t, M_r, 2, num_syms/2, num_subcarriers]
    Eigen::array<int, 5> reshape_dims = {4, M_r, 2, num_syms_half, num_subcarriers};
    CTensor5D h_reshaped = Eigen::TensorMap<CTensor5D>(h_transposed.data(), reshape_dims);
    // h_avg has shape [N_t, M_r, num_syms/2, num_subcarriers]
    CTensor4D h_avg = (h_reshaped.chip(0, 2) + h_reshaped.chip(1, 2)) / gr_complex(2.0f, 0.0f);

    // the input r should be of shape (M_r, num_syms, num_subcarriers) where 2 represents consecutive symbols
    // the input h should be of shape (4, M_r, num_syms_half, num_subcarriers) where 4 represents N_t
    // y: [2, num_syms, num_subcarriers], gains: [2, num_syms, num_subcarriers]
    auto [y, gains] = alamouti_decode_zf_double(r_reshaped, h_avg);

    CTensor2D comparison = (gains.chip(0, 0) >= gains.chip(1, 0)).cast<gr_complex>();
    CTensor4D cluster0isbetter = Eigen::TensorMap<CTensor4D>(
        comparison.data(), Eigen::array<int, 4>{1, 1, num_syms, num_subcarriers});
    comparison = (gains.chip(0, 0) < gains.chip(1, 0)).cast<gr_complex>();
    CTensor4D cluster1isbetter = Eigen::TensorMap<CTensor4D>(
        comparison.data(), Eigen::array<int, 4>{1, 1, num_syms, num_subcarriers});
    // shape = (num_syms_half * 2, num_subcarriers)

    CTensor2D y_chipped = y.chip(0, 0);
    CTensor4D x0 = remap_4d(
        Eigen::TensorMap<CTensor4D>(y_chipped.data(), Eigen::array<int, 4>{1, 1, num_syms, num_subcarriers}),
        d_modtype
    );

    y_chipped = y.chip(1, 0);
    CTensor4D x1 = remap_4d(
        Eigen::TensorMap<CTensor4D>(y_chipped.data(), Eigen::array<int, 4>{1, 1, num_syms, num_subcarriers}),
        d_modtype
    ); // TODO: Use modulator2 here

    CTensor4D better_x = cluster0isbetter * x0 + cluster1isbetter * x1;
    Eigen::array<Eigen::Index, 4> offsets = {0, 0, 0, 0};
    Eigen::array<Eigen::Index, 4> extents = {h.dimension(0), 2, h.dimension(2), h.dimension(3)};
    CTensor4D channel0 = h.slice(offsets, extents);
    offsets = {0, 2, 0, 0};
    CTensor4D channel1 = h.slice(offsets, extents);

    CTensor2D better_x_reshaped = Eigen::TensorMap<CTensor2D>(
        better_x.data(), Eigen::array<int, 2>{num_syms, num_subcarriers});
    CTensor4D better_x_alamouti = Eigen::TensorMap<CTensor4D>(
        alamouti_encode(better_x_reshaped).data(),
        Eigen::array<int, 4>{2, 1, num_syms, num_subcarriers}); // {2, 1 , num_syms , num_subcarriers}

    CTensor4D channel_better_cluster = (
        channel0 * cluster0isbetter.broadcast(Eigen::array<int, 4>{M_r, 2, 1, 1}) +
            channel1 * cluster1isbetter.broadcast(Eigen::array<int, 4>{M_r, 2, 1, 1})
    ); // {M_r, 2, num_syms, num_subcarriers}
    CTensor4D channel_worse_cluster = (
        channel0 * cluster1isbetter.broadcast(Eigen::array<int, 4>{M_r, 2, 1, 1}) +
            channel1 * cluster0isbetter.broadcast(Eigen::array<int, 4>{M_r, 2, 1, 1})
    ); // {M_r, 2, num_syms, num_subcarriers}

    CTensor4D better_cluster_effect = Eigen::TensorMap<CTensor4D>(
        matmul_4d(channel_better_cluster, better_x_alamouti).data(),
        Eigen::array<int, 4>{M_r, 2, num_syms_half, num_subcarriers}
    ); // {M_r, 2, num_syms_half, num_subcarriers}

    // Remove the effect form the received signal
    CTensor4D ry_all_new = r_reshaped - better_cluster_effect; // {M_r, 2, num_syms_half, num_subcarriers}
    Eigen::array<int, 4> stride_array = {1, 1, 2, 1}; // Choose every second element in the num_syms dimension
    offsets = {0, 0, 1, 0};
    extents = {M_r, 2, num_syms - 1, num_subcarriers};
    CTensor4D channel_worse_cluster_reshaped = (channel_worse_cluster.stride(stride_array) +
        channel_worse_cluster.slice(offsets, extents).stride(stride_array))
        / gr_complex(2, 0); // {M_r, 2, num_syms/2, num_subcarriers}

    auto [new_y, new_SNR] =
        alamouti_decode(ry_all_new, channel_worse_cluster_reshaped.shuffle(Eigen::array<int, 4>{1, 0, 2, 3}));
    new_y = new_y / (new_SNR.cast<gr_complex>());
    CTensor4D new_y_reshaped =
        Eigen::TensorMap<CTensor4D>(new_y.data(), Eigen::array<int, 4>{1, 1, num_syms, num_subcarriers});

    // CTensor4D new_x =  new_y_reshaped;
    // TODO: Make this modulator1 and modulator2 if the modulation order is different across clusters
    x0 = cluster0isbetter * better_x + cluster1isbetter * new_y_reshaped;
    x1 = cluster1isbetter * better_x + cluster0isbetter * new_y_reshaped;

    CTensor3D equalized_x(2, num_syms, num_subcarriers);
    equalized_x.chip(0, 0) = x0.chip(0, 0).chip(0, 0);
    equalized_x.chip(1, 0) = x1.chip(0, 0).chip(0, 0);

    dout << "output size: " << equalized_x.dimensions() << std::endl;

    // symbols & csi output
    for (int m = 0; m < d_numsyms; m++)
    {
        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;
            // output has row-major layout, shape  (num_syms, num_sc)
            int offset = m * d_scdata + sc_cnt;
            out0[offset] = equalized_x(0, m, i);
            out1[offset] = equalized_x(1, m, i);
            sc_cnt += 1;
        }
    }

    noutput_items = (d_scdata * d_numsyms);
    for (int ch = 0; ch < 2; ch++)
        add_item_tag(ch,
                     nitems_written(ch),
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(noutput_items),
                     pmt::string_to_symbol(name()));

    return noutput_items;
}

} /* namespace gr::ncjt */
