/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_RG_DEMAPPER_IMPL_H
#define INCLUDED_NCJT_RG_DEMAPPER_IMPL_H

#include "ctrl.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/ncjt/rg_demapper.h>
#include <gnuradio/tagged_stream_block.h>
#include <vector>

namespace gr
{
  namespace ncjt
  {

    class rg_demapper_impl : public rg_demapper
    {
    private:
      // We leave everything mostly unchanged, but add support for FFT=64 or 256
      int d_phase;
      bool d_ctrl_ok; // whether control was successfully decoded
      int d_code_rate;
      int d_code_rate_phase1;
      int d_code_rate_phase2;
      int d_code_rate_phase3;

      int d_modtype;
      int d_modtype_phase1;
      int d_modtype_phase2;
      int d_modtype_phase3;

      int d_nstrm_param;
      int d_nstrm_phase1;
      int d_nstrm_phase2;
      int d_nstrm_phase3;

      int d_data_checksum;
      int d_seqno;
      bool d_tag_snr;
      bool d_debug;
      int cc;
      bool d_usecsi;
      int d_sd_num;
      int d_n_ofdm_syms;

      // The new code just picks d_fftsize based on d_sd_num
      int d_fftsize;

      CTRL d_ctrl_obj;

      
      uint64_t d_extended;
      unsigned __int128 d_raw_ctrl;

      void update_seqno();
      void update_coding_rate();
      void update_modtype();
      void update_data_checksum();

    protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items) override;

    public:
      rg_demapper_impl(int phase,
                       int rgmode,
                       int nstrm,
                       bool usecsi,
                       bool tag_snr,
                       bool debug);
      ~rg_demapper_impl() override {}

      int work(int noutput_items, gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items) override;
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_RG_DEMAPPER_IMPL_H */
