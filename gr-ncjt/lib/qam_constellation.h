/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef GR_NCJT_LIB_QAM_CONSTELLATION_H
#define GR_NCJT_LIB_QAM_CONSTELLATION_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/block.h>

namespace gr::ncjt
{

extern const std::vector<gr_complex> CONST_QPSK; // QPSK constellation
extern const std::vector<gr_complex> CONST_16QAM; // 16QAM constellation
extern const std::vector<gr_complex> CONST_64QAM; // 64QAM constellation
extern const std::vector<gr_complex> CONST_256QAM; // 256QAM constellation

}

#endif //GR_NCJT_LIB_QAM_CONSTELLATION_H
