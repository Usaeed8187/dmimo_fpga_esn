/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_RG_MAPPER_IMPL_H
#define INCLUDED_NCJT_RG_MAPPER_IMPL_H

#include <cmath>
#include <complex>
#include <vector>
#include <gnuradio/ncjt/rg_mapper.h>
#include <gnuradio/tagged_stream_block.h>
#include <stdexcept>

namespace gr
{
  namespace ncjt
  {

    class rg_mapper_impl : public rg_mapper
    {
    private:
      // ---------------------------------------------------------------------
      // Existing parameters
      // ---------------------------------------------------------------------
      int d_nstrm;                     // number of streams
      int d_n_ofdm_syms;               // total OFDM symbols for data
      int d_sc_num;                    // total subcarriers used (56 or 242, etc.)
      std::vector<int> d_pilot_sc_ind; // pilot subcarrier indices
      bool d_add_cyclic_shift;         // flag to apply shift
      bool d_debug;                    // debug flag
      int cc;                          // Debug counter

      // ---------------------------------------------------------------------
      // New from the updated mapper
      // ---------------------------------------------------------------------
      int d_fftsize;                           // can be 64 or 256
      int d_npt;                               // number of pilot subcarriers per symbol (4 or 8)
      int d_numue;                             // number of UEs (for multi-user CPT)
      int d_ueidx;                             // index of this UE
      bool d_mucpt;                            // use orthogonal CPT pilots for multi-user
      bool d_add_ltf;                          // whether to prepend LTF data
      std::vector<gr_complex> d_phaseshift[4]; // for cyclic shift of up to 4 streams

      uint16_t d_seqno; 

      // LTF data
      int d_ltfdata_len;      // length of the LTF samples per stream
      int d_nltfsyms;         // number of LTF OFDM symbols
      gr_complex *d_ltf_data; // buffer for the LTF data (if any)

      // Pre‐generated CPT pilots for the entire OFDM frame
      float *d_cpt_pilot; // size: d_npt * d_nstrm * d_n_ofdm_syms

      // Derived
      int d_data_syms_per_stream; // total data subcarriers * n_ofdm_syms
      int d_total_symbols_required;

      // ---------------------------------------------------------------------
      // Internal helpers
      // ---------------------------------------------------------------------
      void generate_cpt_pilots();
      uint64_t read_ltf_data(const char *filename);

    protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items) override;

    public:
      rg_mapper_impl(int nstrm,
                        int n_ofdm_syms,
                        int sc_num,
                        const std::vector<int> &pilot_sc_ind,
                        bool addcs,
                        bool debug,
                        int numue,
                        int ueidx,
                        bool mucpt,
                        bool addltf,
                        const char *ltfdata);
      ~rg_mapper_impl() override;

      void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;
      int work(int noutput_items, gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items) override;
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_RG_MAPPER_IMPL_H */
