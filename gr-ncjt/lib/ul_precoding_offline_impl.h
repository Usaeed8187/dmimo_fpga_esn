/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UL_PRECODING_OFFLINE_IMPL_H
#define INCLUDED_NCJT_UL_PRECODING_OFFLINE_IMPL_H

#include <gnuradio/ncjt/ul_precoding_offline.h>
// #include <gnuradio/ncjt/ul_precoding.h>
#include <gnuradio/message.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace gr::ncjt
{

class ul_precoding_offline_impl : public ul_precoding_offline 
{
private:
  const int SC_NUM = 56; // Number of valid subcarriers
  int d_nss;  // Number of spatial streams
  int d_ul_ntx;  // Number of uplink transmitter antennas
  int d_dl_ntx;  // Number of downlink transmitter antennas
  int d_dl_nrx;  // Number of downlink transmitter antennas
  int d_num_symbols;
  int d_num_precoded_syms;
  bool d_eigenmode_precoding;
  uint64_t d_channel_idx = 0;
  int d_csi_data_len;

  gr_complex *d_map_matrix;  // spatial mapping matrix (Nt, Nss)

  bool d_debug;

  gr_complex *d_csi_data;
  uint64_t read_csi_data(const char *filename);

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

  int calculate_output_stream_length(const gr_vector_int &ninput_items);

  void
  process_csi_message(const pmt::pmt_t &msg);

  void
  update_svd_precoding(gr_complex *csidata);

  void
  update_eigenmode_precoding(gr_complex *csidata);

public:
  ul_precoding_offline_impl(int nss,int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms,
                            int numprecodedsyms, bool eigenmode,
                            const char *filename, bool debug);
  ~ul_precoding_offline_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_UL_PRECODING_OFFLINE_IMPL_H */

