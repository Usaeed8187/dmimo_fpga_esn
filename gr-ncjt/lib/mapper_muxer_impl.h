/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_MAPPER_MUXER_IMPL_H
#define INCLUDED_NCJT_MAPPER_MUXER_IMPL_H

#include <gnuradio/ncjt/mapper_muxer.h>
#include "qam_constellation.h"
#include "common.h"
#include "ctrl.h"

#include <set>

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <deque>
#include <array>


namespace gr {
  namespace ncjt {
    class mapper_muxer_impl : public mapper_muxer {
    private:
      // Block parameters
      int d_nstrm; ///< Number of parallel streams
      int d_phase1_modtype; ///< Bits per symbol (2,4,6,8)
      int d_phase2_modtype;
      int d_phase3_modtype;
      int d_n_ofdm_syms; ///< Number of OFDM symbols per frame
      int d_sd_num; ///< Number of data subcarriers per OFDM symbol
      bool d_use_polar; ///< Polar encoding flag (for control)
      int d_code_rate; ///< Code rate index for data (0=no coding, 1..7 per table)
      bool d_deterministic_input;
      bool d_debug; ///< Debug flag

      std::set<int> d_valid_mod_types = {2, 4, 6, 8};

      int cc; // Debug counter

      uint16_t d_seqno; // increments by 1 each packet

      // QAM constellation references
      const std::vector<gr_complex> *d_constellation_qpsk;
      const std::vector<gr_complex> *d_constellation_16qam;
      const std::vector<gr_complex> *d_constellation_64qam;
      const std::vector<gr_complex> *d_constellation_256qam;

      // LDPC encoder + rate matcher
      std::unique_ptr<srsran::ldpc_encoder> d_ldpc_encoder;
      std::unique_ptr<srsran::ldpc_rate_matcher> d_ldpc_matcher;

      std::vector<uint8_t> d_bit_buffer;

      // --- network‑producer state ---
      std::thread           d_producer_thr;
      std::atomic<bool>     d_producer_run{false};

      std::atomic<bool>     d_video_on{false};
      // timestamp of last successful packet reception
      std::chrono::steady_clock::time_point d_last_packet_time;

      std::deque<std::array<uint8_t, 188>> d_net_fifo;
      std::mutex            d_net_mtx;       // protects d_net_fifo

      // for incoming‐rate printing
      std::chrono::steady_clock::time_point d_rate_last_time;
      size_t                                d_rate_byte_count;

      // multicast params – make them ctor arguments if you like
      std::string           d_mcast_grp = "239.255.0.1";
      uint16_t              d_mcast_port = 1234;

      void producer_loop();
      void start_producer();
      void stop_producer();

      std::chrono::steady_clock::time_point d_last_warn_time;  // for rate‐limit warning

    public:
      mapper_muxer_impl(int rgmode,
                        int nstrm,
                        int phase1_modtype,
                        int phase2_modtype,
                        int phase3_modtype,
                        int code_rate,
                        bool deterministic_input,
                        bool debug);

      ~mapper_muxer_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items) override;
    };
  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_MAPPER_MUXER_IMPL_H */
