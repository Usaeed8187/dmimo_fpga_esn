#include <srsran/channel_coding_factories.h>
#include <srsran/ldpc/ldpc_decoder.h>
#include <srsran/ldpc/ldpc_encoder.h>
#include <srsran/ldpc/ldpc_encoder_buffer.h>
#include <srsran/ldpc/codeblock_metadata.h>
#include <srsran/ldpc/ldpc_base_graph.h>
#include <srsran/support/units.h>
#include "srsran/srsvec/bit.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <cassert>
#include <random>
#include <memory>
#include <vector>

using namespace srsran;

// Global LDPC objects (initialized in main)
static std::unique_ptr<ldpc_encoder> d_ldpc_encoder;
static std::unique_ptr<ldpc_rate_matcher> d_ldpc_matcher;
static std::unique_ptr<ldpc_decoder> d_ldpc_decoder;
static std::unique_ptr<ldpc_rate_dematcher> d_ldpc_dematcher;

/**
 * @brief Decodes the LDPC encoded bits.
 *
 * @param message_length The length of the original message.
 * @param llrs           The vector of log-likelihood ratios corresponding to the encoded codeword.
 * @return std::vector<uint8_t> The decoded message as a vector of bits.
 */
std::vector<uint8_t> ldpc_decode(int message_length, const std::vector<log_likelihood_ratio> &llrs)
{
    // Compute lifting size using the original message length.
    auto d_lifting_size = ldpc::compute_lifting_size_BG1(units::bits(message_length));

    // Set up the codeblock metadata configuration.
    codeblock_metadata d_cfg;
    d_cfg.tb_common = {ldpc_base_graph_type::BG1, static_cast<ldpc::lifting_size_t>(d_lifting_size)};
    d_cfg.tb_common.mod = modulation_scheme_from_string("QPSK");
    int d_padded_message_length = d_lifting_size * 22;
    d_cfg.cb_specific.nof_filler_bits = d_padded_message_length - message_length;

    // Decoder configuration setup.
    ldpc_decoder::configuration d_cfg_dec;
    d_cfg_dec = {{d_cfg.tb_common}, {}};

    // Allocate memory for the dematched LLRs.
    std::vector<log_likelihood_ratio> llrs_dematched;
    llrs_dematched.resize(d_lifting_size * 66);

    // Buffers to hold the decoded bits.
    dynamic_bit_buffer d_decoded;
    std::vector<uint8_t> d_decoded_bits_unpacked;
    d_decoded.resize(d_padded_message_length);
    d_decoded_bits_unpacked.resize(d_padded_message_length);

    // Rate dematch the LLRs.
    d_ldpc_dematcher->rate_dematch(llrs_dematched, llrs, true, d_cfg);
    // Decode the codeword.
    d_ldpc_decoder->decode(d_decoded, llrs_dematched, nullptr, d_cfg_dec);
    // Unpack the decoded bits.
    srsvec::bit_unpack(d_decoded_bits_unpacked, d_decoded);

    // Remove the filler bits to retrieve the original message.
    d_decoded_bits_unpacked.resize(message_length);

    return d_decoded_bits_unpacked;
}

/**
 * @brief Encodes the message using LDPC.
 *
 * @param message         The original message as a vector of bits.
 * @param codeword_length The length of the codeword after encoding.
 * @return std::vector<uint8_t> The encoded codeword as a vector of bits.
 */
std::vector<uint8_t> ldpc_encode(const std::vector<uint8_t> &message, int codeword_length)
{
    // Determine the original message length.
    int message_length = message.size();

    // Compute the lifting size for BG1 using the message length (in bits).
    auto d_lifting_size = ldpc::compute_lifting_size_BG1(units::bits(message_length));

    // Set up the codeblock metadata configuration.
    codeblock_metadata d_cfg;
    d_cfg.tb_common = {ldpc_base_graph_type::BG1, static_cast<ldpc::lifting_size_t>(d_lifting_size)};
    d_cfg.tb_common.mod = modulation_scheme_from_string("QPSK");
    // Calculate the padded message length (22 is the number of rows for BG1)
    auto d_padded_message_length = d_lifting_size * 22;
    d_cfg.cb_specific.nof_filler_bits = d_padded_message_length - message_length;

    // Allocate buffers for packed message and rate matching.
    dynamic_bit_buffer d_padded_message_packed;
    d_padded_message_packed.resize(d_padded_message_length);

    dynamic_bit_buffer d_rm_buffer;
    d_rm_buffer.resize(codeword_length);

    // Allocate output vector for the unpacked (encoded) codeword.
    std::vector<uint8_t> d_unpacked;
    d_unpacked.resize(codeword_length);

    // Create a copy of the message and pad it with filler bits.
    std::vector<uint8_t> padded_message = message;
    for (size_t i = 0; i < d_cfg.cb_specific.nof_filler_bits; ++i)
        padded_message.push_back(0);

    // Pack the padded message bits.
    srsvec::bit_pack(d_padded_message_packed, padded_message);

    // Perform LDPC encoding and rate matching.
    const ldpc_encoder_buffer &encode_buffer = d_ldpc_encoder->encode(d_padded_message_packed, d_cfg.tb_common);
    d_ldpc_matcher->rate_match(d_rm_buffer, encode_buffer, d_cfg);

    // Unpack the final codeword bits.
    srsvec::bit_unpack(d_unpacked, d_rm_buffer);

    return d_unpacked;
}

int main(int argc, char **argv)
{
    // Create LDPC objects (software-based) using the available factories.
    auto encoder_factory = srsran::create_ldpc_encoder_factory_sw("avx2");
    auto matcher_factory = srsran::create_ldpc_rate_matcher_factory_sw();
    auto decoder_factory = srsran::create_ldpc_decoder_factory_sw("avx2");
    auto dematcher_factory = srsran::create_ldpc_rate_dematcher_factory_sw("avx2");

    assert(encoder_factory && matcher_factory);
    d_ldpc_encoder = encoder_factory->create();
    d_ldpc_matcher = matcher_factory->create();

    assert(decoder_factory && dematcher_factory);
    d_ldpc_decoder = decoder_factory->create();
    d_ldpc_dematcher = dematcher_factory->create();

    // Define test configurations: {codeword_length, message_length}
    std::vector<std::pair<int, int>> test_configs = {
        {2016, 1000},
        {2080, 1000},
        {8096, 6000},
        {12096, 10080}};

    for (auto [codeword_length, message_length] : test_configs)
    {
        std::cout << "\nTesting with codeword_length = " << codeword_length
                  << " and message_length = " << message_length << std::endl;

        // Generate a random message (sequence of 0s and 1s).
        std::vector<uint8_t> message(message_length);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 1);
        for (int i = 0; i < message_length; i++)
        {
            message[i] = dis(gen);
        }

        // Encode the message.
        std::vector<uint8_t> encoded_message = ldpc_encode(message, codeword_length);

        // Print message details.
        std::cout << "Message Length: " << message_length << std::endl;
        std::cout << "Codeword Length: " << codeword_length << std::endl;
        std::cout << "Encoded Message Length: " << encoded_message.size() << std::endl;

        // Flip two bits to test the decoder (if possible).
        if (encoded_message.size() > 200)
        {
            encoded_message[50] = !encoded_message[50];
            encoded_message[200] = !encoded_message[200];
        }

        // Convert the encoded bits into LLRs for the decoder.
        std::vector<log_likelihood_ratio> llrs(encoded_message.size());
        for (size_t i = 0; i < encoded_message.size(); i++)
        {
            llrs[i] = (encoded_message[i] == 0) ? 100 : -100;
        }

        // Decode the encoded message.
        std::vector<uint8_t> decoded_message = ldpc_decode(message_length, llrs);

        // Verify if the decoded message matches the original.
        if (decoded_message == message)
        {
            std::cout << "Decoding successful: decoded message matches the original." << std::endl;
        }
        else
        {
            std::cout << "Decoding failed: decoded message does NOT match the original." << std::endl;
        }
    }

    return 0;
}
