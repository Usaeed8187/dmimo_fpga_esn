/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UL_PRECODING_IMPL_H
#define INCLUDED_NCJT_UL_PRECODING_IMPL_H

#include <gnuradio/ncjt/ul_precoding.h>
#include <gnuradio/message.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace gr::ncjt
{

class ul_precoding_impl : public ul_precoding
{
private:
    const int SC_NUM = 56; // number of valid subcarriers
    int d_nstrms;
    int d_num_symbols;

    gr_complex *d_map_matrix;

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    process_csi_message(const pmt::pmt_t &msg);

    void
    apply_direct_mapping(gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items);

    void
    apply_mapping_2tx(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);

    void
    apply_mapping_4tx(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);

public:
    ul_precoding_impl(int nstrms, int numsyms, bool debug);
    ~ul_precoding_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_UL_PRECODING_IMPL_H */
