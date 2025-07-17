#ifndef INCLUDED_COMMON_NCJT_H
#define INCLUDED_COMMON_NCJT_H

#include <vector>
#include <cstdint>
#include <srsran/channel_coding_factories.h>
#include <gnuradio/gr_complex.h>

#if defined(_MSC_VER)
#  define NCJT_FQN   __FUNCSIG__
#else
#  define NCJT_FQN   __PRETTY_FUNCTION__
#endif

// helper that removes "return‑type " prefix and "(...)" suffix
inline std::string _ncjt_pretty_strip(const char* pretty)
{
    std::string fn(pretty);

    // 1) kill the argument list  (…)   → “…::foo”
    auto pos_paren = fn.find('(');
    if (pos_paren != std::string::npos) fn.erase(pos_paren);

    // 2) drop the return type up to the last space  “void …::foo” → “…::foo”
    auto pos_space = fn.rfind(' ');
    if (pos_space != std::string::npos) fn.erase(0, pos_space + 1);

    return fn;
}

#define NCJT_LOG(EN, STREAM)                                                    \
    do {                                                                        \
        if (EN) {                                                               \
            std::cout << "[" << _ncjt_pretty_strip(NCJT_FQN) << "] "            \
                      << STREAM << std::endl;                                   \
        }                                                                       \
    } while (0)

namespace gr {
  namespace ncjt {
    /**
   * @brief Convert number-of-bits per symbol (2,4,6,8) into the
   *        compact index (0..3) used in the control header.
   * @throws std::runtime_error if bits is not one of {2,4,6,8}.
   */
    inline int modtype_bits_to_index(int bits) {
      switch (bits) {
        case 2: return 0; // QPSK
        case 4: return 1; // 16QAM
        case 6: return 2; // 64QAM
        case 8: return 3; // 256QAM
        default: throw std::runtime_error("modtype_bits_to_index: invalid bits");
      }
    }

    /**
     * @brief Convert the compact index (0..3) back into the
     *        number-of-bits per symbol (2,4,6,8).
     * @throws std::runtime_error if idx is not in [0..3].
     */
    inline int modtype_index_to_bits(int idx) {
      switch (idx) {
        case 0: return 2; // QPSK
        case 1: return 4; // 16QAM
        case 2: return 6; // 64QAM
        case 3: return 8; // 256QAM
        default: throw std::runtime_error("modtype_index_to_bits: invalid index");
      }
    }

    ////////////////////////////////////////////////////////////////////
    // Code rates array
    ////////////////////////////////////////////////////////////////////
    static const double code_rates[] = {
      1.0, // 0 => no coding
      1.0 / 4, // 1 => 1/4
      1.0 / 3, // 2 => 1/3
      1.0 / 2, // 3 => 1/2
      2.0 / 3, // 4 => 2/3
      3.0 / 4, // 5 => 3/4
      4.0 / 5, // 6 => 4/5
      5.0 / 6 // 7 => 5/6
    };

    ///////////////////////////////////////////////////////////////////////
    // CRC16 computation declaration
    ///////////////////////////////////////////////////////////////////////
    uint16_t compute_crc16(const uint8_t *bits,
                           int length);

    ///////////////////////////////////////////////////////////////////////
    // SNR Quantization and De-quantization
    ///////////////////////////////////////////////////////////////////////

    const int RB_SIZE = 13; // Number of subcarriers in a resource block

    const int B0 = 8; // 8 bits for the first SNR
    const int Bd = 5; // 5 bits for each subsequent SNR
    // Ranges for the absolute value and the differential
    const float MIN_ABS = -1.0f;
    const float MAX_ABS = 40.0f;
    const float MIN_DIFF = -6.0f; // use tighter range for differences
    const float MAX_DIFF = 6.0f;

    uint64_t quantize_snrs(const float *snr_values,
                           std::size_t num_snr_elements,
                           int num_initial_bits,
                           int num_diff_bits,
                           float min_value,
                           float max_value,
                           float diff_min_value,
                           float diff_max_value);

    void dequantize_snrs(uint64_t packed_bits,
                         std::size_t num_snr_elements,
                         int num_initial_bits,
                         int num_diff_bits,
                         float min_value,
                         float max_value,
                         float diff_min_value,
                         float diff_max_value,
                         float *out_snr_values);

    ///////////////////////////////////////////////////////////////////////
    // LDPC encode and decode functions
    ///////////////////////////////////////////////////////////////////////
    std::vector<uint8_t> ldpc_encode(const std::vector<uint8_t> &message,
                                     int codeword_length,
                                     srsran::ldpc_encoder *ldpc_encoder,
                                     srsran::ldpc_rate_matcher *ldpc_matcher);

    std::vector<uint8_t> ldpc_decode(int message_length,
                                     const std::vector<srsran::log_likelihood_ratio> &llrs,
                                     srsran::ldpc_decoder *ldpc_decoder,
                                     srsran::ldpc_rate_dematcher *ldpc_dematcher);

    ///////////////////////////////////////////////////////////////////////
    // Hard-decision threshold demapping for one symbol.
    ///////////////////////////////////////////////////////////////////////
    gr_complex demap_symbol(float x,
                            float y,
                            float csi_val,
                            int rx_modtype,
                            uint8_t *outbits);
  } // namespace ncjt
} // namespace gr
#endif // INCLUDED_COMMON_NCJT_H
