/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_P1_PRECODING_IMPL_H
#define INCLUDED_NCJT_P1_PRECODING_IMPL_H

#include <gnuradio/ncjt/p1_precoding.h>
#include <gnuradio/message.h>
#include "cmatrix.h"
#include <vector>
#include <array>


namespace gr {
namespace ncjt {

class p1_precoding_impl : public p1_precoding {
private:

    int d_nfft;     // OFDM FFT size
    int d_num_sc;   // Number of valid subcarriers
    int d_nss;  // Number of uplink spatial streams
    int d_ntx;  // Number of uplink transmitter antennas = number of downlink receive antennas
    int d_ntx_gnb;  // Number of downlink transmitter antennas
    int d_nrx_ue;   // Number of downlink receive antennas
    int d_num_symbols; // Total number of OFDM symbols
    bool d_wideband; // Use wideband precoding method
    int d_logfreq;
    int d_total_frames;
    int d_num_ue;
    gr_complex *d_map_matrix;   // spatial mapping matrix (Nt, Nss)

    int d_num_iter; // number of iterations for weighted mean precoding scheme
    std::vector<CMatrixX> d_pmi_ue; // Last received PMI matrix per UE (N_UE x Nss x Nt)
    std::vector<std::vector<float>>  d_cqi_ue;  // Last received CQI values per stream per UE

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
    process_pmi_message(const pmt::pmt_t &msg);

    void
    process_cqi_message(const pmt::pmt_t &msg);

    void
    update_p1_weighted_mean_precoding();

public:
  p1_precoding_impl(int num_ue, int nss, int ntx_gnb, int num_iter, int numltfsyms, int numdatasyms,
                    bool wideband, int nfft, int num_sc, int logfreq,
                    bool debug);
  ~p1_precoding_impl();

  int work(int noutput_items, gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_P1_PRECODING_IMPL_H */
