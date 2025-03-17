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

void
mc_stbc_decode_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int ch = 0; ch < d_nrx; ch++)
    {
        add_item_tag(ch,
                     offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len / 2),
                     pmt::string_to_symbol(name()));
        add_item_tag(ch,
                     offset + packet_len / 2,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len / 2),
                     pmt::string_to_symbol(name()));
    }
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

    int min_input_items = std::min(ninput_items[0], ninput_items[1]);
    if (min_input_items != d_scnum * (d_ntx + d_numsyms))
    {
        dout << "Input data length: " << ninput_items[0] << std::endl;
        throw std::runtime_error("Input data length incorrect!");
    }
    int ninput_syms = min_input_items / d_scnum;

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

    // Get CPE estimation tags
    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_phi1"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_phi1 tag");
    d_cpe_phi1 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_offset1"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_offset1 tag");
    d_cpe_offset1 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_phi2"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_phi2 tag");
    d_cpe_phi2 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_offset2"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_offset2 tag");
    d_cpe_offset2 = pmt::to_float(d_tags[0].value);

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
    CTensor4D ch = chant_est.broadcast(bcast);  // (num_rx, num_tx, num_syms, num_sc)
    dout << "ch size: " << ch.dimensions() << std::endl;

    // CPE compensation
    CTensor4D cpe_comp_all(d_nrx, d_ntx, d_numsyms, d_scnum);
    for (int symidx = 0; symidx < d_numsyms; symidx++)
    {
        float cpe_phi1 = -(d_cpe_offset1 + symidx * d_cpe_phi1);
        gr_complex cpe_comp1 = std::exp(gr_complex(0, cpe_phi1));
        float cpe_phi2 = -(d_cpe_offset2 + symidx * d_cpe_phi2);
        gr_complex cpe_comp2 = std::exp(gr_complex(0, cpe_phi2));
        for (int m = 0; m < d_scnum; m++)
        {
            for (int n = 0; n < 2; n++)
            {
                cpe_comp_all(0, n, symidx, m) = cpe_comp1;
                cpe_comp_all(1, n, symidx, m) = cpe_comp1;
            }
            for (int n = 2; n < 4; n++)
            {
                cpe_comp_all(0, n, symidx, m) = cpe_comp2;
                cpe_comp_all(1, n, symidx, m) = cpe_comp2;
            }
        }
    }
    Eigen::array<int, 4> cpe_dims = {d_scnum, d_nrx, 2, d_numsyms / 2};
    auto cpe_comp_frame = Eigen::TensorMap<CTensor4D>(cpe_comp_all.data(), cpe_dims);
    ch *= cpe_comp_frame;

    int M_r = r.dimension(0);
    int num_syms = r.dimension(1);
    int num_subcarriers = r.dimension(2);
    int num_syms_half = num_syms / 2;

    CTensor4D r_reshaped = Eigen::TensorMap<CTensor4D>(
        r.data(), Eigen::array<int, 4>{M_r, 2, num_syms_half, num_subcarriers});

    Eigen::array<int, 4> transpose_dims = {1, 0, 2, 3}; // Transpose to [N_t, M_r, num_syms, num_subcarriers]
    CTensor4D h_transposed = ch.shuffle(transpose_dims);

    // Reshape to [N_t, M_r, 2, num_syms/2, num_subcarriers]
    Eigen::array<int, 5> reshape_dims = {4, M_r, 2, num_syms_half, num_subcarriers};
    CTensor5D h_reshaped = Eigen::TensorMap<CTensor5D>(h_transposed.data(), reshape_dims);
    // h_avg has shape [N_t, M_r, num_syms/2, num_subcarriers]
    CTensor4D h_avg = (h_reshaped.chip(0, 2) + h_reshaped.chip(1, 2)) / gr_complex(2.0f, 0.0f);

    // the input r should be of shape (M_r, num_syms, num_subcarriers) where 2 represents consecutive symbols
    // the input h should be of shape (4, M_r, num_syms_half, num_subcarriers) where 4 represents N_t
    // y: [2, num_syms, num_subcarriers], gains: [2, num_syms, num_subcarriers]
    auto [y, gains] = alamouti_decode_zf_double(r_reshaped, h_avg);

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
            out0[offset] = y(0, m, i);
            out1[offset] = y(1, m, i);
            sc_cnt += 1;
        }
    }

    noutput_items = (d_scdata * d_numsyms);
    add_packet_tag(nitems_written(0), noutput_items);

    return noutput_items;
}

} /* namespace gr::ncjt */
