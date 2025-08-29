/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_DEMAPPER_IMPL_H
#define INCLUDED_NCJT_DEMAPPER_IMPL_H

#include <gnuradio/ncjt/demapper.h>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <vector>
#include <queue>
#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <array>
#include "common.h"
#include "qam_constellation.h"

// A simple structure to keep track of a pending packet we need to decode.
struct pending_packet_t
{
  uint64_t start_input_offset;
  int pkt_len_syms;
  int rx_modtype;
  int rx_nstrm;

  int syms_collected;
  std::vector<gr_complex> data_buf;
  std::vector<gr_complex> csi_buf;
};

namespace gr
{
  namespace ncjt
  {

    class demapper_impl : public demapper
    {
    private:
      bool d_output_raw;
      bool d_deterministic_input;
      bool d_debug; // Debug flag
      int cc;       // Debug counter

      int d_rgmode; // RG mode (0-7)
      int d_current_phase;    // Storing received rx_current_phase tag value.
      int d_last_reserved;    // The most recently seen "rx_reserved"
      int d_last_nstrm;       // The most recently seen "rx_nstrm"
      int d_last_modtype;     // The most recently seen "rx_modtype"
      int d_last_modtype_phase1; // The most recently seen "rx_modtype_phase1"
      int d_last_modtype_phase2; // The most recently seen "rx_modtype_phase2"
      int d_last_modtype_phase3; // The most recently seen "rx_modtype_phase3"
      unsigned __int128 d_last_raw_ctrl; // The most recently seen "rx_raw_ctrl"
      int d_last_coding_rate; // The most recently seen "rx_coding_rate"
      int d_last_pkt_len;     // The most recently seen "packet_len"
      int d_last_seq_no;      // The most recently seen "rx_seqno"
      int d_last_syms_per_stream;
      bool d_last_ctrl_ok; // The most recently seen "rx_ctrl_ok"

      // For CRC checking
      bool d_found_rx_checksum;
      uint64_t d_rx_data_checksum;

      uint8_t *d_coded_buf;
      int d_coded_len;

      uint8_t *d_info_buf;
      int d_info_len;

      std::vector<float> d_snr_rbs_db;

      std::unique_ptr<srsran::ldpc_decoder> d_ldpc_decoder;
      std::unique_ptr<srsran::ldpc_rate_dematcher> d_ldpc_dematcher;

      // LDPC encoder + rate matcher (only needed if d_deterministic_input)
      std::unique_ptr<srsran::ldpc_encoder> d_ldpc_encoder;
      std::unique_ptr<srsran::ldpc_rate_matcher> d_ldpc_matcher;

      // Hard-decision threshold demapping for one symbol.
      // outbits[0..(rx_modtype-1)] gets the bits.
      gr_complex demap_symbol(float x, float y, float csi_val,
                              int rx_modtype, uint8_t *outbits) const;


    public:
      demapper_impl(int rgmode,
                    bool coded,
                    bool deterministic_input,
                    bool debug);
      ~demapper_impl();

      // Standard block interface
      void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;
      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items) override;
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_DEMAPPER_IMPL_H */
