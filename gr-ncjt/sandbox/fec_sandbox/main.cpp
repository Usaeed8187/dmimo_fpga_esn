#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>

// Include the polar encoder header from GNU Radio FEC.
#include <gnuradio/fec/polar_encoder.h>
#include <gnuradio/fec/polar_decoder_sc.h>

// A known reliability ordering for N=128.
// Indices are listed from least reliable (index 0) to most reliable (index 127).
static const int RELIABILITY_SEQUENCE_128[128] = {
    0, 64, 32, 96, 16, 80, 48, 112, 8, 72, 40, 104, 24, 88, 56, 120,
    4, 68, 36, 100, 20, 84, 52, 116, 12, 76, 44, 108, 28, 92, 60, 124,
    2, 66, 34, 98, 18, 82, 50, 114, 10, 74, 42, 106, 26, 90, 58, 122,
    6, 70, 38, 102, 22, 86, 54, 118, 14, 78, 46, 110, 30, 94, 62, 126,
    1, 65, 33, 97, 17, 81, 49, 113, 9, 73, 41, 105, 25, 89, 57, 121,
    5, 69, 37, 101, 21, 85, 53, 117, 13, 77, 45, 109, 29, 93, 61, 125,
    3, 67, 35, 99, 19, 83, 51, 115, 11, 75, 43, 107, 27, 91, 59, 123,
    7, 71, 39, 103, 23, 87, 55, 119, 15, 79, 47, 111, 31, 95, 63, 127};

int main()
{
    // Define polar code parameters:
    // N must be a power of 2.
    int block_size = 128;
    // K is the number of information bits.
    int num_info_bits = 64;

    // (block_size - num_info_bits) = 64 frozen bits.

    // -----------------------------------------------------
    // 1) Select frozen bit positions from the reliability sequence
    //    The first (N - K) indices in RELIABILITY_SEQUENCE_128 are the *least* reliable,
    //    so we freeze them. The remaining 64 are used for data.
    // -----------------------------------------------------
    std::vector<int> frozen_positions(block_size - num_info_bits);
    for (int i = 0; i < block_size - num_info_bits; i++)
    {
        frozen_positions[i] = RELIABILITY_SEQUENCE_128[i];
    }

    // All frozen bits set to 0:
    std::vector<uint8_t> frozen_values(frozen_positions.size(), 0);

    // Create the polar encoder object.
    // The 'false' flag indicates we are using "unpacked" bits (one bit per byte).
    auto encoder = gr::fec::code::polar_encoder::make(
        block_size,
        num_info_bits,
        frozen_positions,
        frozen_values,
        false // unpacked
    );

    // Retrieve the expected input and output sizes.
    int input_size = encoder->get_input_size();   // This should be 64
    int output_size = encoder->get_output_size(); // This should be 128

    std::cout << "Polar Encoder Setup:" << std::endl;
    std::cout << "  Input size (bits): " << input_size << std::endl;
    std::cout << "  Output size (bits): " << output_size << std::endl;

    // Prepare an example information word (64 bits).
    // Since the encoder expects unpacked bits (one bit per byte),
    // we create an input vector of length 'input_size'.
    std::vector<unsigned char> input(input_size);
    for (int i = 0; i < input_size; i++)
    {
        input[i] = i % 2; // Some trivial pattern
    }

    // Allocate an output buffer for the encoded bits (128).
    std::vector<unsigned char> output(output_size, 0);

    // Run the encoder.
    encoder->generic_work(input.data(), output.data());

    // Display the input bits (for debugging).
    std::cout << "Input bits: ";
    for (auto bit : input)
    {
        std::cout << (int)bit << " ";
    }
    std::cout << std::endl;

    // Display the encoded bits (for debugging).
    std::cout << "Encoded bits: ";
    for (auto bit : output)
    {
        std::cout << (int)bit << " ";
    }
    std::cout << std::endl;

    // Convert the hard bits into LLRs for the decoder.
    // For a polar decoder, typically:
    //   bit == 0 -> LLR = -10 (strong belief in 0)
    //   bit == 1 -> LLR = +10 (strong belief in 1)
    std::vector<float> soft_input(output_size);
    for (int i = 0; i < output_size; i++)
    {
        soft_input[i] = (output[i] == 0) ? -10.0f : 10.0f;
    }

    // Create a polar decoder object with SC (successive cancellation).
    // We must pass the same frozen_positions and frozen_values used by the encoder.
    auto decoder = gr::fec::code::polar_decoder_sc::make(
        block_size,
        num_info_bits,
        frozen_positions,
        frozen_values);

    // Prepare a buffer for the decoder output bits (64).
    std::vector<unsigned char> decoded(input_size, 0);

    // Decode the LLRs.
    decoder->generic_work(soft_input.data(), decoded.data());

    // Display the decoded bits (for debugging).
    std::cout << "Decoded bits: ";
    for (auto bit : decoded)
    {
        std::cout << (int)bit << " ";
    }
    std::cout << std::endl;

    // Check if the decoded bits match the original input.
    bool match = true;
    for (int i = 0; i < input_size; i++)
    {
        if (decoded[i] != input[i])
        {
            match = false;
            break;
        }
    }
    std::cout << "Decoded bits match input? " << (match ? "Yes" : "No") << std::endl;

    return 0;
}
