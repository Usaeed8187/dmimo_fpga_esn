#include "hll.h"
#include <cmath>
#include <set>
#include <algorithm>
#include <limits>
#include <omp.h>
#include <chrono>
#include <iostream>

// Constructor and methods implementation
HardLogLikelihoodVanilla::HardLogLikelihoodVanilla(const std::complex<float>* constellation_points,
                                                   int num_points,
                                                   int bits_per_symbol,
                                                   bool sum_over_rx,
                                                   bool return_bit_llrs)
        : bits_per_symbol_(bits_per_symbol),
          sum_over_rx_(sum_over_rx),
          return_bit_llrs_(return_bit_llrs)
{
    constellation_.assign(constellation_points, constellation_points + num_points);

    // Extract unique real and imaginary parts
    std::set<float> unique_reals, unique_imags;
    for (auto& s : constellation_) {
        unique_reals.insert(s.real());
        unique_imags.insert(s.imag());
    }
    candidate_reals_.assign(unique_reals.begin(), unique_reals.end());
    candidate_imags_.assign(unique_imags.begin(), unique_imags.end());

    // Compute midpoints
    auto computeMidpoints = [](const std::vector<float>& values) {
        std::vector<float> midpoints;
        for (size_t i = 0; i + 1 < values.size(); i++)
            midpoints.push_back((values[i] + values[i+1]) / 2.0f);
        return midpoints;
    };

    std::vector<float> midpoints_real = computeMidpoints(candidate_reals_);
    std::vector<float> midpoints_imag = computeMidpoints(candidate_imags_);

    // Initialize left and right bounds for real parts
    left_bounds_real_.resize(candidate_reals_.size());
    right_bounds_real_.resize(candidate_reals_.size());
    left_bounds_real_[0] = -std::numeric_limits<float>::infinity();
    for (size_t i = 1; i < candidate_reals_.size(); i++)
        left_bounds_real_[i] = midpoints_real[i-1];
    for (size_t i = 0; i + 1 < candidate_reals_.size(); i++)
        right_bounds_real_[i] = midpoints_real[i];
    right_bounds_real_.back() = std::numeric_limits<float>::infinity();

    // Initialize left and right bounds for imaginary parts
    left_bounds_imag_.resize(candidate_imags_.size());
    right_bounds_imag_.resize(candidate_imags_.size());
    left_bounds_imag_[0] = -std::numeric_limits<float>::infinity();
    for (size_t i = 1; i < candidate_imags_.size(); i++)
        left_bounds_imag_[i] = midpoints_imag[i-1];
    for (size_t i = 0; i + 1 < candidate_imags_.size(); i++)
        right_bounds_imag_[i] = midpoints_imag[i];
    right_bounds_imag_.back() = std::numeric_limits<float>::infinity();
}

void HardLogLikelihoodVanilla::Compute(const float* rx_real,
                                       const float* rx_imag,
                                       const float* SNRs,
                                       int M, int N,
                                       std::vector<float>& out_values,
                                       int num_threads) {
    if (num_threads > 0) {
        omp_set_num_threads(num_threads);
    }
    int num_candidates = (int) constellation_.size();
    int total_symbols = M * N; // total received symbols

    // Compute sqrt(SNR) for each symbol
    std::vector<float> sqrt_SNR(total_symbols);
    #pragma omp parallel for
    for (int i = 0; i < total_symbols; i++) {
        sqrt_SNR[i] = std::sqrt(SNRs[i]);
    }

    // Determine intervals for each symbol
    std::vector<int> real_intervals(total_symbols), imag_intervals(total_symbols);
    #pragma omp parallel for
    for (int idx = 0; idx < total_symbols; idx++) {
        float r = rx_real[idx];
        float i = rx_imag[idx];
        real_intervals[idx] = findInterval(right_bounds_real_.data(), (int)right_bounds_real_.size(), r);
        imag_intervals[idx] = findInterval(right_bounds_imag_.data(), (int)right_bounds_imag_.size(), i);
    }

    // Precompute a_i, b_i, c_i, d_i
    std::vector<float> a_i(total_symbols), b_i(total_symbols);
    std::vector<float> c_i(total_symbols), d_i(total_symbols);
    #pragma omp parallel for
    for (int idx = 0; idx < total_symbols; idx++) {
        a_i[idx] = left_bounds_real_[ real_intervals[idx] ];
        b_i[idx] = right_bounds_real_[ real_intervals[idx] ];
        c_i[idx] = left_bounds_imag_[ imag_intervals[idx] ];
        d_i[idx] = right_bounds_imag_[ imag_intervals[idx] ];
    }

    // Compute probabilities
    std::vector<float> prob(M * N * num_candidates, 0.0f);
    #pragma omp parallel for collapse(2)
    for (int m = 0; m < M; m++) {
        for (int n = 0; n < N; n++) {
            int idx = m*N + n;
            float sn = sqrt_SNR[idx];
            for (int c = 0; c < num_candidates; c++) {
                float reC = constellation_[c].real();
                float imC = constellation_[c].imag();

                float val_real_right = (b_i[idx] - reC)*sn;
                float val_real_left  = (a_i[idx] - reC)*sn;
                float val_imag_right = (d_i[idx] - imC)*sn;
                float val_imag_left  = (c_i[idx] - imC)*sn;

                float real_part = 0.25f * (std::erf(val_real_right) - std::erf(val_real_left)) *
                                  (std::erf(val_imag_right) - std::erf(val_imag_left));

                prob[m*N*num_candidates + n*num_candidates + c] = real_part;
            }
        }
    }

    if (!return_bit_llrs_) {
        // return_bit_llrs == false
        // Compute log_prob
        std::vector<float> log_prob(M * N * num_candidates, -std::numeric_limits<float>::infinity());
        #pragma omp parallel for
        for (int i = 0; i < M*N*num_candidates; i++) {
            if (prob[i] > 0.0f)
                log_prob[i] = std::log(prob[i]);
        }

        if (sum_over_rx_) {
            // sum_over_rx_syms = true, return_bit_llrs = false
            // sum over the N dimension
            std::vector<float> log_prob_summed(M * num_candidates, -std::numeric_limits<float>::infinity());

            // Each (m, c) is an independent output — safe to parallelize
            #pragma omp parallel for collapse(2)
            for (int m = 0; m < M; m++) {
                for (int c = 0; c < num_candidates; c++) {
                    float sum_val = 0.0f;
                    bool first = true;
                    for (int n = 0; n < N; n++) {
                        float val = log_prob[m*N*num_candidates + n*num_candidates + c];
                        if (first) {
                            sum_val = val;
                            first = false;
                        } else {
                            sum_val += val;
                        }
                    }
                    log_prob_summed[m*num_candidates + c] = sum_val;
                }
            }

            // Find most likely symbol index
            out_values.resize(2*M);
            #pragma omp parallel for
            for (int m = 0; m < M; m++) {
                int best_c = 0;
                float best_val = log_prob_summed[m*num_candidates];
                for (int c = 1; c < num_candidates; c++) {
                    if (log_prob_summed[m*num_candidates + c] > best_val) {
                        best_val = log_prob_summed[m*num_candidates + c];
                        best_c = c;
                    }
                }
                out_values[2*m]   = constellation_[best_c].real();
                out_values[2*m+1] = constellation_[best_c].imag();
            }
        } else {
            // sum_over_rx_syms = false, return_bit_llrs = false
            out_values = log_prob;
        }

    } else {
        // return_bit_llrs = true
        float tiny = std::numeric_limits<float>::min();
        float add_val = std::pow(tiny, 1.0f / (float)N);

        // Multiply across N for each (m, c)
        std::vector<float> prob_multiplied(M * num_candidates, 1.0f);
        #pragma omp parallel for collapse(2)
        for (int m = 0; m < M; m++) {
            for (int c = 0; c < num_candidates; c++) {
                float accum = 1.0f;
                for (int n = 0; n < N; n++) {
                    float val = prob[m*N*num_candidates + n*num_candidates + c] + add_val;
                    accum *= val;
                }
                prob_multiplied[m*num_candidates + c] = accum;
            }
        }

        out_values.resize(M * bits_per_symbol_);
        #pragma omp parallel for
        for (int m = 0; m < M; m++) {
            std::vector<float> llr_temp(bits_per_symbol_, 0.0f);

            for (int bit = bits_per_symbol_ - 1; bit >= 0; bit--) {
                double sum_bit1 = 0.0;
                double sum_bit0 = 0.0;
                for (int c = 0; c < num_candidates; c++) {
                    int bit_val = (c >> bit) & 1;
                    if (bit_val == 1)
                        sum_bit1 += (double)prob_multiplied[m*num_candidates + c];
                    else
                        sum_bit0 += (double)prob_multiplied[m*num_candidates + c];
                }

                float llr_val = (std::log((float)sum_bit1) - std::log((float)sum_bit0)) / (std::log(2.0));
                llr_temp[bits_per_symbol_ - 1 - bit] = llr_val;
            }

            for (int bit = 0; bit < bits_per_symbol_; bit++) {
                out_values[m*bits_per_symbol_ + bit] = llr_temp[bit];
            }
        }
    }
}
