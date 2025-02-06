/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ul_precoding_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <Eigen/Dense>
#include "utils.h"

namespace gr::ncjt
{

ul_precoding::sptr
ul_precoding::make(int nstrms, int numsyms, bool debug)
{
    return gnuradio::make_block_sptr<ul_precoding_impl>(nstrms, numsyms, debug);
}

ul_precoding_impl::ul_precoding_impl(int nstrms, int numsyms, bool debug)
    : gr::tagged_stream_block(
    "ul_precoding",
    gr::io_signature::make(1, 1, nstrms * sizeof(gr_complex)),
    gr::io_signature::make(nstrms, nstrms, sizeof(gr_complex)), "packet_len"),
      d_num_symbols(numsyms), d_debug(debug)
{
    if (nstrms < 1 || nstrms > 8)
        throw std::runtime_error("only support 1 to 8 streams");
    d_nstrms = nstrms;

    d_map_matrix = malloc_complex(d_nstrms * d_nstrms * SC_NUM);
    for (int k = 0; k < SC_NUM; k++)
        for (int m = 0; m < d_nstrms; m++)
            for (int n = 0; n < d_nstrms; n++)
                d_map_matrix[k * d_nstrms * d_nstrms + d_nstrms * m + n] = (n % d_nstrms == m) ? 1.0 : 0.0;

    message_port_register_in(pmt::mp("csi"));
    set_msg_handler(pmt::mp("csi"), [this](const pmt::pmt_t &msg) { process_csi_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

ul_precoding_impl::~ul_precoding_impl()
{
    if (d_map_matrix != nullptr)
        volk_free(d_map_matrix);
}

int
ul_precoding_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = d_num_symbols * SC_NUM;
    return noutput_items;
}

void
ul_precoding_impl::process_csi_message(const pmt::pmt_t &msg)
{
}

int
ul_precoding_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    int num_symbols = ninput_items[0] / SC_NUM;
    if (num_symbols != d_num_symbols)
        throw std::runtime_error("input data length not correct");

    // Apply spatial mapping
    if (d_nstrms == 2)
        apply_direct_mapping(input_items, output_items);
    else
        apply_mapping_4tx(input_items, output_items);

    return d_num_symbols * SC_NUM;
}

void
ul_precoding_impl::apply_direct_mapping(gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    for (int n = 0; n < d_num_symbols; n++)
    {
        for (int k = 0; k < SC_NUM; k++)
        {
            int offset = n * d_nstrms * SC_NUM + d_nstrms * k;
            out0[n * SC_NUM + k] = in[offset];
            out1[n * SC_NUM + k] = in[offset + 1];
        }
    }
}

void
ul_precoding_impl::apply_mapping_2tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    for (int n = 0; n < d_num_symbols; n++)
    {
        int mtx_size = d_nstrms * d_nstrms;
        Eigen::Matrix2cf Y(d_nstrms, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            Eigen::Map<const Eigen::Matrix2cf> X(&in[n * d_nstrms * SC_NUM + d_nstrms * k], d_nstrms, 1); // (Nss, 1)
            Eigen::Map<Eigen::Matrix2cf> Q(&d_map_matrix[mtx_size * k], d_nstrms, d_nstrms); // (Nt, Nss)
            Y = Q * X;
            out0[n * SC_NUM + k] = Y(0, 0);
            out1[n * SC_NUM + k] = Y(1, 0);
        }
    }
}

void
ul_precoding_impl::apply_mapping_4tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];
    auto out2 = (gr_complex *) output_items[2];
    auto out3 = (gr_complex *) output_items[3];

    // Apply spatial mapping
    for (int n = 0; n < d_num_symbols; n++)
    {
        Eigen::Matrix4cf Y(4, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            Eigen::Map<const Eigen::Matrix4cf> X(&in[n * d_nstrms * SC_NUM + d_nstrms * k], 4, 1); // (Nss, 1)
            Eigen::Map<Eigen::Matrix4cf> Q(&d_map_matrix[16 * k], 4, 4); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            out0[n * SC_NUM + k] = Y(0, 0);
            out1[n * SC_NUM + k] = Y(1, 0);
            out2[n * SC_NUM + k] = Y(2, 0);
            out3[n * SC_NUM + k] = Y(3, 0);
        }
    }
}

} /* namespace gr::ncjt */
