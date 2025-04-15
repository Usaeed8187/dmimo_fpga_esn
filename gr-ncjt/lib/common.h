#ifndef INCLUDED_COMMON_NCJT_H
#define INCLUDED_COMMON_NCJT_H

#include <vector>
#include <cstdint>
#include <srsran/channel_coding_factories.h>
#include <gnuradio/gr_complex.h>

namespace gr
{
    namespace ncjt
    {
        ////////////////////////////////////////////////////////////////////
        // Code rates array
        ////////////////////////////////////////////////////////////////////
        static const double code_rates[] = {
            1.0,     // 0 => no coding
            1.0 / 4, // 1 => 1/4
            1.0 / 3, // 2 => 1/3
            1.0 / 2, // 3 => 1/2
            2.0 / 3, // 4 => 2/3
            3.0 / 4, // 5 => 3/4
            4.0 / 5, // 6 => 4/5
            5.0 / 6  // 7 => 5/6
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
