/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_UL_PRECODING_IMPL_H
#define INCLUDED_NCJT_UL_PRECODING_IMPL_H

#include <gnuradio/ncjt/ul_precoding.h>
#include <gnuradio/message.h>
#include "cmatrix.h"

namespace gr::ncjt
{

class ul_precoding_impl : public ul_precoding
{
private:
    int d_fftsize;  // OFDM FFT size
    int d_scnum;  // Number of valid subcarriers
    int d_nss;  // Number of uplink spatial streams
    int d_ntx;  // Number of uplink transmitter antennas = number of downlink receive antennas
    int d_ntx_gnb;  // Number of downlink transmitter antennas
    int d_nrx_ue;   // Number of downlink receive antennas
    int d_num_symbols; // Total number of OFDM symbols
    bool d_wideband; // Use wideband precoding method
    bool d_loadcsi; // Load offline CSI at the beginning

    bool d_eigenmode_precoding;  // Use eigen-mode precoding instead of SVD precoding
    gr_complex *d_csi_data;  // buffer for CSI data (Nt, Nr)
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
    update_svd_precoding(const gr_complex *csidata);

    void
    update_eigenmode_precoding(const gr_complex *csidata);

    int
    read_offline_csi(const char *filename, int csidlen);

public:
    ul_precoding_impl(int fftsize, int nss, int ntx, int ntx_gnb, int numltfsyms, int numdatasyms,
                      bool eigenmode, bool wideband, bool loadcsi, const char *csifile, bool debug);
    ~ul_precoding_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_UL_PRECODING_IMPL_H */
