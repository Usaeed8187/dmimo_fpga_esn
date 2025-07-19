/* -*- c++ -*- */
/*
 * Copyright 2025.
 *
 * A simple pass-through “no-air” block that can optionally remove
 * pilot subcarriers so that the demapper sees 52-subcarrier frames
 * even though rg_mapper created 56-subcarrier frames.
 *
 * Now extended to add AWGN based on an snr_db parameter.
 */

#ifndef INCLUDED_NCJT_NOAIR_IMPL_H
#define INCLUDED_NCJT_NOAIR_IMPL_H

#include <gnuradio/ncjt/noair.h>
#include <chrono>
#include <random>

namespace gr
{
  namespace ncjt
  {

    class noair_impl : public noair
    {
    private:
      // Our constructor parameters:
      int d_ofdm_syms;                 // number of OFDM symbols
      int d_sc_num;                    // number of subcarriers (56)
      std::vector<int> d_pilot_sc_ind; // indices of pilot subcarriers
      bool d_debug;                    // Debug flag
      int cc;                          // Debug counter

      int d_num_drop_init_packets; // number of packets to drop
      uint16_t d_seqno; 

      float d_frame_per_sec;
      double d_seconds_per_frame;
      std::chrono::steady_clock::time_point d_last_frame_time;

      float d_snr_db;     // user-supplied SNR in dB
      std::mt19937 d_rng; // RNG for noise
      std::normal_distribution<float> d_norm;

      pmt::pmt_t d_me; // block identifier for tags

    protected:
      // The base class calls this to figure out how many output samples we produce:
      int calculate_output_stream_length(const gr_vector_int &ninput_items) override;

    public:
      noair_impl(int rgmode,
                 float frame_per_sec,
                 float snr_db,
                 int num_drop_init_packets,
                 bool debug);
      ~noair_impl() override;

      int work(int noutput_items,
               gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items) override;
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_NOAIR_IMPL_H */
