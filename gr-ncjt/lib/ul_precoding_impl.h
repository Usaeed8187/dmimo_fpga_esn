/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
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
    const int SC_NUM = 56; // Number of valid subcarriers
    int d_nss;  // Number of spatial streams
    int d_ntx;  // Number of transmitter antennas
    int d_num_symbols;
    int d_num_precoded_syms;
    bool d_eigenmode_precoding;

    gr_complex *d_map_matrix;  // spatial mapping matrix (Nt, Nss)

    bool d_debug;

protected:
    boost::mutex fp_mutex;

    void
    apply_mapping_2tx(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);

    void
    apply_mapping_4tx(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);

    void
    apply_mapping_ntx(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);

    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    process_csi_message(const pmt::pmt_t &msg);

    void
    update_svd_precoding(gr_complex *csidata);

    void
    update_eigenmode_precoding(gr_complex *csidata);

public:
    ul_precoding_impl(int nss, int ntx, int numhtsyms, int numdatasyms, int numprecodedsyms, bool eigenmode, bool debug);
    ~ul_precoding_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_UL_PRECODING_IMPL_H */
