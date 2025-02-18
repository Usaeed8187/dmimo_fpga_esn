/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef GR_NCJT_LIB_STBC_SIC_DECODER_H
#define GR_NCJT_LIB_STBC_SIC_DECODER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/block.h>

#include <unsupported/Eigen/CXX11/Tensor>
#include <bitset>
#include <vector>
#include <string>
#include <complex>
#include <cmath>
#include <algorithm>
#include <cassert>

#include "cmatrix.h"

CTensor3D
alamouti_encode(CTensor2D &input);

std::tuple<CTensor2D, Tensor2D>
alamouti_decode(const CTensor4D &r, const CTensor4D &h);

std::pair<CTensor3D, Tensor3D>
alamouti_decode_zf_double(const CTensor4D &r, const CTensor4D &h);

std::pair<CTensor3D, Tensor3D>
alamouti_decode_zf_sic_double(CTensor3D &r, CTensor4D &h, const int num_bits_per_symbol);

#endif //GR_NCJT_LIB_STBC_SIC_DECODER_H
