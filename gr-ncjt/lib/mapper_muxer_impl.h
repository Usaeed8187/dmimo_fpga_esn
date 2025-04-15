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

namespace gr
{
    namespace ncjt
    {

        class mapper_muxer_impl : public mapper_muxer
        {
        private:
            // Block parameters
            int d_nstrm;        ///< Number of parallel streams
            int d_modtype;      ///< Bits per symbol (2,4,6,8)
            int d_n_ofdm_syms;  ///< Number of OFDM symbols per frame
            int d_sd_num;       ///< Number of data subcarriers per OFDM symbol
            bool d_use_polar;   ///< Polar encoding flag (for control)
            int d_code_rate;    ///< Code rate index for data (0=no coding, 1..7 per table)
            bool d_deterministic_input;
            bool d_debug;       ///< Debug flag

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

            std::vector<uint8_t> ldpc_encode(const std::vector<uint8_t> &message, int codeword_length);

        public:
            mapper_muxer_impl(int nstrm,
                              int modtype,
                              int n_ofdm_syms,
                              int sd_num,
                              bool use_polar,
                              int code_rate,
                              bool deterministic_input,
                              bool debug);
            ~mapper_muxer_impl() override {}

            void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

            int general_work(int noutput_items,
                             gr_vector_int &ninput_items,
                             gr_vector_const_void_star &input_items,
                             gr_vector_void_star &output_items) override;
        };

    } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_MAPPER_MUXER_IMPL_H */
