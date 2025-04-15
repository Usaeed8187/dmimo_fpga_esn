#include "common.h"
#include <algorithm> // added for std::clamp
#include <cmath>     // added for std::round
#include "srsran/srsvec/bit.h"
#include "qam_constellation.h"

namespace gr
{
    namespace ncjt
    {

        ///////////////////////////////////////////////////////////////////////
        // CRC16 computation definition
        ///////////////////////////////////////////////////////////////////////
        uint16_t compute_crc16(const uint8_t *bits, int length)
        {
            uint16_t crc = 0xFFFF;
            for (int i = 0; i < length; i++)
            {
                uint8_t in_bit = bits[i] & 1;
                bool mix = ((crc >> 15) & 1) ^ in_bit;
                crc <<= 1;
                if (mix)
                    crc ^= 0x1021;
            }
            return crc;
        }

        ///////////////////////////////////////////////////////////////////////
        // SNR Quantization and De-quantization
        ///////////////////////////////////////////////////////////////////////

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
            const double norm = ((std::fmax(lo, std::fmin(hi, x))) - lo) / (hi - lo);
            const uint64_t code = static_cast<uint64_t>(std::llround(norm * range));
            return (code > range) ? range : code; // guard against rounding overflow
        }

        static inline double decode(uint64_t code, double lo, double hi, int bits)
        {
            const uint64_t range = code_range(bits);
            const double norm = static_cast<double>(code) / static_cast<double>(range);
            return lo + norm * (hi - lo);
        }

        /* ------------------------------------------------------------------ */
        /*  quantize_snrs                                                     */
        /* ------------------------------------------------------------------ */
        uint64_t quantize_snrs(const float *snr_values,
                               std::size_t num_snr_elements,
                               int num_initial_bits,
                               int num_diff_bits,
                               float min_value,
                               float max_value,
                               float diff_min_value,
                               float diff_max_value)
        {
            // Parameter checks
            if (!snr_values)
                throw std::invalid_argument("snr_values is nullptr");
            if (num_snr_elements == 0)
                throw std::invalid_argument("num_snr_elements == 0");
            if (min_value >= max_value)
                throw std::invalid_argument("min_value >= max_value");
            if (diff_min_value >= diff_max_value)
                throw std::invalid_argument("diff_min_value >= diff_max_value");
            if (num_initial_bits <= 0 || num_initial_bits > 64)
                throw std::invalid_argument("num_initial_bits must be in (0,64]");
            if (num_diff_bits <= 0 || num_diff_bits > 64)
                throw std::invalid_argument("num_diff_bits must be in (0,64]");

            const std::size_t total_bits =
                static_cast<std::size_t>(num_initial_bits) +
                static_cast<std::size_t>(num_diff_bits) * (num_snr_elements - 1);

            if (total_bits > 64)
                throw std::runtime_error("Not enough space: the sequence needs more than 64 bits");

            // Quantisation
            uint64_t packed = 0;
            int shift = 0;

            // First (absolute) SNR value
            {
                const uint64_t code = encode(snr_values[0],
                                             static_cast<double>(min_value),
                                             static_cast<double>(max_value),
                                             num_initial_bits);
                packed |= code << shift;
                shift += num_initial_bits;
            }

            // Subsequent differential values
            for (std::size_t i = 1; i < num_snr_elements; ++i)
            {
                const double diff = static_cast<double>(snr_values[i]) - static_cast<double>(snr_values[i - 1]);

                const uint64_t code = encode(diff,
                                             static_cast<double>(diff_min_value),
                                             static_cast<double>(diff_max_value),
                                             num_diff_bits);
                packed |= code << shift;
                shift += num_diff_bits;
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
        void dequantize_snrs(uint64_t packed_bits,
                             std::size_t num_snr_elements,
                             int num_initial_bits,
                             int num_diff_bits,
                             float min_value,
                             float max_value,
                             float diff_min_value,
                             float diff_max_value,
                             float *out_snr_values)
        {
            // Parameter checks
            if (!out_snr_values)
                throw std::invalid_argument("out_snr_values is nullptr");
            if (num_snr_elements == 0)
                throw std::invalid_argument("num_snr_elements == 0");
            if (min_value >= max_value)
                throw std::invalid_argument("min_value >= max_value");
            if (diff_min_value >= diff_max_value)
                throw std::invalid_argument("diff_min_value >= diff_max_value");
            if (num_initial_bits <= 0 || num_initial_bits > 64)
                throw std::invalid_argument("num_initial_bits must be in (0,64]");
            if (num_diff_bits <= 0 || num_diff_bits > 64)
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
                const uint64_t mask = code_range(num_initial_bits);
                const uint64_t code = (packed_bits >> shift) & mask;
                const double value = decode(code,
                                            static_cast<double>(min_value),
                                            static_cast<double>(max_value),
                                            num_initial_bits);
                out_snr_values[0] = static_cast<float>(value);
                shift += num_initial_bits;
            }

            // Remaining values (differential)
            for (std::size_t i = 1; i < num_snr_elements; ++i)
            {
                const uint64_t mask = code_range(num_diff_bits);
                const uint64_t code = (packed_bits >> shift) & mask;
                const double diff = decode(code,
                                           static_cast<double>(diff_min_value),
                                           static_cast<double>(diff_max_value),
                                           num_diff_bits);

                out_snr_values[i] = out_snr_values[i - 1] + static_cast<float>(diff);
                shift += num_diff_bits;
            }
        }

        ///////////////////////////////////////////////////////////////////////
        // LDPC encode and decode functions
        ///////////////////////////////////////////////////////////////////////
        std::vector<uint8_t> ldpc_encode(const std::vector<uint8_t> &message,
                                         int codeword_length,
                                         srsran::ldpc_encoder *ldpc_encoder,
                                         srsran::ldpc_rate_matcher *ldpc_matcher)
        {
            // Determine the original message length.
            int message_length = message.size();

            // Compute the lifting size for BG1 using the message length (in bits).
            auto lifting_size = srsran::ldpc::compute_lifting_size_BG1(
                srsran::units::bits(message_length));

            // Set up the codeblock metadata configuration.
            srsran::codeblock_metadata cfg;
            cfg.tb_common = {srsran::ldpc_base_graph_type::BG1,
                             static_cast<srsran::ldpc::lifting_size_t>(lifting_size)};
            cfg.tb_common.mod = srsran::modulation_scheme_from_string("QPSK");
            // Calculate the padded message length (22 is the number of rows for BG1)
            auto padded_message_length = lifting_size * 22;
            cfg.cb_specific.nof_filler_bits = padded_message_length - message_length;

            // Allocate buffers for packed message and rate matching.
            srsran::dynamic_bit_buffer padded_message_packed;
            padded_message_packed.resize(padded_message_length);

            srsran::dynamic_bit_buffer rm_buffer;
            rm_buffer.resize(codeword_length);

            // Allocate output vector for the unpacked (encoded) codeword.
            std::vector<uint8_t> unpacked;
            unpacked.resize(codeword_length);

            // Create a copy of the message and pad it with filler bits.
            std::vector<uint8_t> padded_message = message;
            for (unsigned int i = 0; i < cfg.cb_specific.nof_filler_bits; ++i)
                padded_message.push_back(0);

            // Pack the padded message bits.
            srsran::srsvec::bit_pack(padded_message_packed, padded_message);

            // Perform LDPC encoding and rate matching.
            const srsran::ldpc_encoder_buffer &encode_buffer =
                ldpc_encoder->encode(padded_message_packed, cfg.tb_common);
            ldpc_matcher->rate_match(rm_buffer, encode_buffer, cfg);

            // Unpack the final codeword bits.
            srsran::srsvec::bit_unpack(unpacked, rm_buffer);

            return unpacked;
        }

        std::vector<uint8_t> ldpc_decode(int message_length,
                                         const std::vector<srsran::log_likelihood_ratio> &llrs,
                                         srsran::ldpc_decoder *ldpc_decoder,
                                         srsran::ldpc_rate_dematcher *ldpc_dematcher)
        {
            // Compute lifting size using the original message length.
            auto lifting_size = srsran::ldpc::compute_lifting_size_BG1(
                srsran::units::bits(message_length));

            // Set up the codeblock metadata configuration.
            srsran::codeblock_metadata cfg;
            cfg.tb_common = {srsran::ldpc_base_graph_type::BG1,
                             static_cast<srsran::ldpc::lifting_size_t>(lifting_size)};
            cfg.tb_common.mod = srsran::modulation_scheme_from_string("QPSK");
            int padded_message_length = lifting_size * 22;
            cfg.cb_specific.nof_filler_bits = padded_message_length - message_length;

            // Decoder configuration setup.
            srsran::ldpc_decoder::configuration cfg_dec;
            cfg_dec = {{cfg.tb_common}, {}};

            // Allocate memory for the dematched LLRs.
            std::vector<srsran::log_likelihood_ratio> llrs_dematched;
            llrs_dematched.resize(lifting_size * 66);

            // Buffers to hold the decoded bits.
            srsran::dynamic_bit_buffer decoded;
            std::vector<uint8_t> decoded_bits_unpacked;
            decoded.resize(padded_message_length);
            decoded_bits_unpacked.resize(padded_message_length);

            // Rate dematch the LLRs.
            ldpc_dematcher->rate_dematch(llrs_dematched, llrs, true, cfg);
            // Decode the codeword.
            ldpc_decoder->decode(decoded, llrs_dematched, nullptr, cfg_dec);
            // Unpack the decoded bits.
            srsran::srsvec::bit_unpack(decoded_bits_unpacked, decoded);

            // Remove the filler bits to retrieve the original message.
            decoded_bits_unpacked.resize(message_length);

            return decoded_bits_unpacked;
        }

        ///////////////////////////////////////////////////////////////////////
        // Hard-decision threshold demapping for one symbol.
        ///////////////////////////////////////////////////////////////////////
        gr_complex demap_symbol(float x, float y, float csi_val, int rx_modtype, uint8_t *outbits)
        {
            gr_complex outsym;
            switch (rx_modtype)
            {
            case 2: // QPSK
            {
                // "bit0 = (y<0)?1:0;  bit1 = (x>=0)?1:0"
                outbits[0] = (y < 0.0f) ? 1 : 0;
                outbits[1] = (x >= 0.0f) ? 1 : 0;
                //
                outsym = CONST_QPSK[(outbits[1] << 1) | outbits[0]];
            }
            break;

            case 4: // 16QAM
            {
                // "xm=abs(x); ym=abs(y); h2 = 2.0/sqrt(10.0)*csi_val;  outbits[...]"
                float xm = std::fabs(x);
                float ym = std::fabs(y);
                float h2 = 2.0f / std::sqrt(10.0f);
                if (csi_val > 1e-6f)
                {
                    h2 *= csi_val;
                }

                outbits[0] = (ym <= h2) ? 1 : 0; // LSB
                outbits[1] = (y < 0.0f) ? 1 : 0;
                outbits[2] = (xm <= h2) ? 1 : 0;
                outbits[3] = (x >= 0.0f) ? 1 : 0; // MSB
                //
                outsym = CONST_16QAM[outbits[3] << 3 | outbits[2] << 2 |
                                     outbits[1] << 1 | outbits[0]];
            }
            break;

            case 6: // 64QAM
            {
                // "xm=abs(x); ym=abs(y); a2=2.0/sqrt(42.0); etc."
                float xm = std::fabs(x);
                float ym = std::fabs(y);
                float a2 = 2.0f / std::sqrt(42.0f);
                if (csi_val > 1e-6f)
                {
                    a2 *= csi_val;
                }

                float h2 = a2;        // threshold for "inner ring"
                float h4 = 2.0f * a2; // threshold for "middle ring"
                float h6 = 3.0f * a2; // threshold for "outer ring"

                outbits[0] = ((ym >= h2) && (ym <= h6)) ? 1 : 0;
                outbits[1] = (ym <= h4) ? 1 : 0;
                outbits[2] = (y <= 0.0f) ? 1 : 0;
                outbits[3] = ((xm >= h2) && (xm <= h6)) ? 1 : 0;
                outbits[4] = (xm <= h4) ? 1 : 0;
                outbits[5] = (x >= 0.0f) ? 1 : 0;
                //
                outsym = CONST_64QAM[outbits[5] << 5 | outbits[4] << 4 |
                                     outbits[3] << 3 | outbits[2] << 2 |
                                     outbits[1] << 1 | outbits[0]];
            }
            break;

            case 8: // 256QAM
            {
                // "xm=abs(x); ym=abs(y); a2=2.0/sqrt(170.0); etc."
                float xm = std::fabs(x);
                float ym = std::fabs(y);
                float a2 = 2.0f / std::sqrt(170.0f);
                if (csi_val > 1e-6f)
                {
                    a2 *= csi_val;
                }

                float h2 = a2;
                float h4 = 2.0f * a2;
                float h8 = 4.0f * a2;

                outbits[0] = (std::fabs(std::fabs(ym - h8) - h4) <= h2) ? 1 : 0;
                outbits[1] = (std::fabs(ym - h8) <= h4) ? 1 : 0;
                outbits[2] = (ym <= h8) ? 1 : 0;
                outbits[3] = (y <= 0.0f) ? 1 : 0;
                outbits[4] = (std::fabs(std::fabs(xm - h8) - h4) <= h2) ? 1 : 0;
                outbits[5] = (std::fabs(xm - h8) <= h4) ? 1 : 0;
                outbits[6] = (xm <= h8) ? 1 : 0;
                outbits[7] = (x >= 0.0f) ? 1 : 0;
                //
                outsym = CONST_256QAM[outbits[7] << 7 | outbits[6] << 6 |
                                      outbits[5] << 5 | outbits[4] << 4 |
                                      outbits[3] << 3 | outbits[2] << 2 |
                                      outbits[1] << 1 | outbits[0]];
            }
            break;

            default:
            {
                // fallback: produce all zeros
                for (int b = 0; b < rx_modtype; b++)
                    outbits[b] = 0;
                outsym = gr_complex(0.0f, 0.0f);
            }
            break;
            }
            return outsym;
        }

    } // namespace ncjt
} // namespace gr
