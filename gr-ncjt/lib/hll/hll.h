#ifndef HLL_H
#define HLL_H

#include <vector>
#include <complex>

class HardLogLikelihoodVanilla {
public:
    HardLogLikelihoodVanilla(const std::complex<float>* constellation_points,
                             int num_points,
                             int bits_per_symbol,
                             bool sum_over_rx = true,
                             bool return_bit_llrs = false);

    void Compute(const float* received_symbols_real,
                 const float* received_symbols_imag,
                 const float* SNRs,
                 int M, int N,
                 std::vector<float>& out_values,
                 int num_threads = 0);

private:
    std::vector<std::complex<float>> constellation_;
    int bits_per_symbol_;
    bool sum_over_rx_;
    bool return_bit_llrs_;

    std::vector<float> candidate_reals_;
    std::vector<float> candidate_imags_;
    std::vector<float> left_bounds_real_;
    std::vector<float> right_bounds_real_;
    std::vector<float> left_bounds_imag_;
    std::vector<float> right_bounds_imag_;

    inline int findInterval(const float* boundaries, int size, float value) const {
        const float* end = boundaries + size;
        const float* it = std::lower_bound(boundaries, end, value);
        int idx = (int)(it - boundaries);
        if (idx == size) idx = size - 1;
        return idx;
    }
};

#endif // HLL_H
