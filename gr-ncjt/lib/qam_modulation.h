/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef GR_NCJT_LIB_QAM_MODULATION_H
#define GR_NCJT_LIB_QAM_MODULATION_H

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

// QAMModulator Class
/**
 * @class QAMModulator
 * @brief Represents a Quadrature Amplitude Modulation (QAM) system.
 *
 * The QAMModulator class provides methods for symbol generation, bitstream modulation,
 * and remapping input data to the nearest QAM symbols in the constellation.
 */
class QAMModulator
{
public:
    // Properties
    /**
     * @brief Number of bits per QAM symbol.
     */
    int k; // Number of bits per symbol
    /**
     * @brief The generated QAM symbol vector.
     *
     * Each element is a complex number representing a QAM symbol in the constellation.
     */
    std::vector<gr_complex> points; // QAM symbols
    /**
     * @brief A sorted vector of unique real parts of the QAM symbols.
     */
    std::vector<float> points_real_sorted;   // Sorted unique real parts
    /**
     * @brief The distance between consecutive points in points_real_sorted.
     */
    float distance_real; // Distance between consecutive points in points_real_sorted
    /**
     * @brief The distance between consecutive points in points_imag_sorted.
     */
    std::vector<float> points_imag_sorted;   // Sorted unique imaginary parts
    /**
     * @brief The distance between consecutive points in points_imag_sorted.
     */
    float distance_imag; // Distance between consecutive points in points_imag_sorted

    std::vector<std::vector<int>> grayCode2DDecimal;
    std::vector<std::vector<std::string>> grayCode2D;

    // Constructor
    QAMModulator(int k, bool normalize_power = true);

    std::vector<gr_complex>
    Modulate(const std::string &bit_stream);

    std::string
    Demodulate(const CTensor1D &input_tensor);

};

#endif //GR_NCJT_LIB_QAM_MODULATION_H
