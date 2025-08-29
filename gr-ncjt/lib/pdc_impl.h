/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_PDC_IMPL_H
#define INCLUDED_NCJT_PDC_IMPL_H

#include <gnuradio/ncjt/pdc.h>
#include "hll/hll.h"
#include <vector>
#include "common.h"
#include "qam_constellation.h"
#include <srsran/channel_coding_factories.h>

#include <array>
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace gr
{
  namespace ncjt
  {

    class pdc_impl : public pdc
    {
    private:
      int d_rgmode;          // RG mode (0-7)
      int d_num_packet_syms;
      int d_num_copies;        // Number of copies
      int d_num_threads;       // Number of threads
      bool d_hll_enabled;      // HLL flag
      bool d_majority_enabled; // Majority voting flag
      int d_expire_ms;         // Expiration time in milliseconds
      bool d_deterministic_input;
      bool d_debug; // Debug flag

      int d_no_inp_cnt;

      struct packet
      {
        gr_complex *buf;
        gr_complex *csi_buf;
        int64_t seqno;
        uint64_t packet_len;       // packet length tag
        int64_t modtype;           // modulation type tag
        int64_t p2modtype;       // phase2 modulation type tag
        int64_t p3modtype;       // phase3 modulation type tag
        int64_t rx_coding_rate;    // receiver coding rate tag
        uint64_t rx_reserved;      // reserved tag
        uint64_t rx_data_checksum; // data checksum tag
        float *snr_rbs_db;         // SNR values per RB in dB
        uint64_t ctrl_syms_len;    // length of control symbols
        uint64_t sd_num;           // SD number
        uint64_t timestamp;        // current timestamp in nanoseconds
      };

      std::deque<std::unordered_map<int, packet>> d_pendings;

      std::unique_ptr<srsran::ldpc_decoder> d_ldpc_decoder;
      std::unique_ptr<srsran::ldpc_rate_dematcher> d_ldpc_dematcher;

      // LDPC encoder + rate matcher (only needed if d_deterministic_input)
      std::unique_ptr<srsran::ldpc_encoder> d_ldpc_encoder;
      std::unique_ptr<srsran::ldpc_rate_matcher> d_ldpc_matcher;

      // HLLs for each modulation type and number of copies
      std::unordered_map<int, std::unordered_map<int, HardLogLikelihoodVanilla *>> d_hlls;

      std::chrono::steady_clock::time_point d_last_warn_time;

    public:
      pdc_impl(int rgmode,
               int combiner,
               int num_copies,
               int expire_ms,
               int num_threads,
               bool deterministic_input,
               bool debug);
      ~pdc_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_PDC_IMPL_H */
