/**********************************************************************
 *  SNR quantisation utilities
 *
 *  – quantize_snrs  : packs a sequence of SNR values into a single
 *                     uint64_t using one absolute value followed by
 *                     differentially‑encoded values.
 *  – dequantize_snrs: reverses the process.
 *
 *  Both functions throw std::invalid_argument on bad parameters and
 *  std::runtime_error if the bit‑budget exceeds 64 bits.
 *********************************************************************/

 #include <cstdint>
 #include <cstddef>
 #include <cmath>
 #include <limits>
 #include <stdexcept>
 #include <iostream>
 
 /* ------------------------------------------------------------------ */
 /*  Helper functions                                              */
 /* ------------------------------------------------------------------ */
 static inline uint64_t code_range(int bits)
 {
     // Range (2^bits - 1) that still works when bits == 64
     return (bits == 64)
            ? std::numeric_limits<uint64_t>::max()
            : ((1ULL << bits) - 1ULL);
 }
 
 static inline uint64_t encode(double x, double lo, double hi, int bits)
 {
     const uint64_t range = code_range(bits);
     const double   norm  = ((std::fmax(lo, std::fmin(hi, x))) - lo) / (hi - lo);
     const uint64_t code  = static_cast<uint64_t>(std::llround(norm * range));
     return (code > range) ? range : code; // guard against rounding overflow
 }
 
 static inline double decode(uint64_t code, double lo, double hi, int bits)
 {
     const uint64_t range = code_range(bits);
     const double   norm  = static_cast<double>(code) / static_cast<double>(range);
     return lo + norm * (hi - lo);
 }
 
 /* ------------------------------------------------------------------ */
 /*  quantize_snrs                                                     */
 /* ------------------------------------------------------------------ */
 uint64_t quantize_snrs(const float* snr_values,
                        std::size_t  num_snr_elements,
                        int          num_initial_bits,
                        int          num_diff_bits,
                        float        min_value,
                        float        max_value,
                        float        diff_min_value,
                        float        diff_max_value)
 {
     // Parameter checks
     if (!snr_values)                       throw std::invalid_argument("snr_values is nullptr");
     if (num_snr_elements == 0)             throw std::invalid_argument("num_snr_elements == 0");
     if (min_value >= max_value)            throw std::invalid_argument("min_value >= max_value");
     if (diff_min_value >= diff_max_value)  throw std::invalid_argument("diff_min_value >= diff_max_value");
     if (num_initial_bits <= 0 || num_initial_bits > 64)
         throw std::invalid_argument("num_initial_bits must be in (0,64]");
     if (num_diff_bits   <= 0 || num_diff_bits   > 64)
         throw std::invalid_argument("num_diff_bits must be in (0,64]");
 
     const std::size_t total_bits =
         static_cast<std::size_t>(num_initial_bits) +
         static_cast<std::size_t>(num_diff_bits) * (num_snr_elements - 1);
 
     if (total_bits > 64)
         throw std::runtime_error("Not enough space: the sequence needs more than 64 bits");
 
     // Quantisation
     uint64_t packed = 0;
     int      shift  = 0;
 
     // First (absolute) SNR value
     {
         const uint64_t code = encode(snr_values[0],
                                      static_cast<double>(min_value),
                                      static_cast<double>(max_value),
                                      num_initial_bits);
         packed |= code << shift;
         shift  += num_initial_bits;
     }
 
     // Subsequent differential values
     for (std::size_t i = 1; i < num_snr_elements; ++i)
     {
         const double diff = static_cast<double>(snr_values[i])
                           - static_cast<double>(snr_values[i - 1]);
 
         const uint64_t code = encode(diff,
                                      static_cast<double>(diff_min_value),
                                      static_cast<double>(diff_max_value),
                                      num_diff_bits);
         packed |= code << shift;
         shift  += num_diff_bits;
     }
 
     // Ensure bits beyond our budget are zeroed out:
     if (total_bits < 64)
     {
         const uint64_t mask = (1ULL << total_bits) - 1ULL;
         packed &= mask;
     }
 
     return packed;
 }
 
 /* ------------------------------------------------------------------ */
 /*  dequantize_snrs                                                   */
 /* ------------------------------------------------------------------ */
 void dequantize_snrs(uint64_t     packed_bits,
                      std::size_t  num_snr_elements,
                      int          num_initial_bits,
                      int          num_diff_bits,
                      float        min_value,
                      float        max_value,
                      float        diff_min_value,
                      float        diff_max_value,
                      float*       out_snr_values)
 {
     // Parameter checks
     if (!out_snr_values)                   throw std::invalid_argument("out_snr_values is nullptr");
     if (num_snr_elements == 0)             throw std::invalid_argument("num_snr_elements == 0");
     if (min_value >= max_value)            throw std::invalid_argument("min_value >= max_value");
     if (diff_min_value >= diff_max_value)  throw std::invalid_argument("diff_min_value >= diff_max_value");
     if (num_initial_bits <= 0 || num_initial_bits > 64)
         throw std::invalid_argument("num_initial_bits must be in (0,64]");
     if (num_diff_bits   <= 0 || num_diff_bits   > 64)
         throw std::invalid_argument("num_diff_bits must be in (0,64]");
 
     const std::size_t total_bits =
         static_cast<std::size_t>(num_initial_bits) +
         static_cast<std::size_t>(num_diff_bits) * (num_snr_elements - 1);
 
     if (total_bits > 64)
         throw std::runtime_error("Bit‑stream longer than 64 bits");
 
     // De-quantisation
     int shift = 0;
 
     // First (absolute) value
     {
         const uint64_t mask  = code_range(num_initial_bits);
         const uint64_t code  = (packed_bits >> shift) & mask;
         const double   value = decode(code,
                                       static_cast<double>(min_value),
                                       static_cast<double>(max_value),
                                       num_initial_bits);
         out_snr_values[0]    = static_cast<float>(value);
         shift += num_initial_bits;
     }
 
     // Remaining values (differential)
     for (std::size_t i = 1; i < num_snr_elements; ++i)
     {
         const uint64_t mask = code_range(num_diff_bits);
         const uint64_t code = (packed_bits >> shift) & mask;
         const double   diff = decode(code,
                                      static_cast<double>(diff_min_value),
                                      static_cast<double>(diff_max_value),
                                      num_diff_bits);
 
         out_snr_values[i] = out_snr_values[i - 1] + static_cast<float>(diff);
         shift += num_diff_bits;
     }
 }
 
 /* ------------------------------------------------------------------ */
 /*  Example usage                                                     */
 /* ------------------------------------------------------------------ */
 int main()
 {
     const float  snr[5] = { 12.3f, 15.0f, 14.4f, 10.0f, 9.8f };
     const size_t N      = 5;
 
     // Bits for the first value and each differential
     const int   B0  = 6;  // 6 bits for the first SNR
     const int   Bd  = 4;  // 4 bits for each subsequent SNR
 
     // Ranges for the absolute value and the differential
     const float MIN_ABS  = -5.0f;
     const float MAX_ABS  = 30.0f;
     const float MIN_DIFF = -5.0f;   // use tighter range for differences
     const float MAX_DIFF =  5.0f;
 
     // Pack
     uint64_t packed = quantize_snrs(snr, N, B0, Bd,
                                     MIN_ABS,  MAX_ABS,
                                     MIN_DIFF, MAX_DIFF);
 
     // Unpack
     float recovered[N];
     dequantize_snrs(packed, N, B0, Bd,
                     MIN_ABS,  MAX_ABS,
                     MIN_DIFF, MAX_DIFF,
                     recovered);
 
     // Print results
     std::cout << "Packed value : 0x" << std::hex << packed << std::dec << "\n";
     std::cout << "Recovered SNR:";
     for (auto v : recovered)
         std::cout << ' ' << v;
     std::cout << '\n';

     /////
     float recovered2[4];
     dequantize_snrs(0x2222d, 4, B0, Bd,
        MIN_ABS,  MAX_ABS,
        MIN_DIFF, MAX_DIFF,
        recovered2);
        std::cout << "Recovered SNR2:";
        for (auto v : recovered2)
            std::cout << ' ' << v;
        std::cout << '\n';
 
     return 0;
 }
