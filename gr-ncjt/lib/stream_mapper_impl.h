/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_STREAM_MAPPER_IMPL_H
#define INCLUDED_NCJT_STREAM_MAPPER_IMPL_H

#include <gnuradio/ncjt/stream_mapper.h>
#include "qam_constellation.h"
#include "common.h"
#include "ctrl.h"

#include <set>

namespace gr::ncjt
{

class stream_mapper_impl : public stream_mapper
{
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

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    stream_mapper_impl(int rgmode, int nstrm, int phase1_modtype,
                       int phase2_modtype, int phase3_modtype, int code_rate,
                       bool deterministic_input, bool debug);
    ~stream_mapper_impl(){};

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_STREAM_MAPPER_IMPL_H */
