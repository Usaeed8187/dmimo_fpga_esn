/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "qam_modulation.h"

// Function to generate 1D Gray code
std::vector<std::string>
generateGrayCode(int k)
{
    if (k == 0)
    {
        return {"0"}; // Base case for 0 bits
    }
    if (k == 1)
    {
        return {"0", "1"}; // Base case for 1 bit
    }

    // Recursive step: Generate Gray code for k-1 bits
    std::vector<std::string> smallerGrayCode = generateGrayCode(k - 1);

    // Generate Gray code for k bits by reflecting the smaller set
    std::vector<std::string> result;

    // Prefix '0' to the original sequence
    for (const auto &code : smallerGrayCode)
    {
        result.push_back("0" + code);
    }

    // Prefix '1' to the reversed sequence
    for (auto it = smallerGrayCode.rbegin(); it != smallerGrayCode.rend(); ++it)
    {
        result.push_back("1" + *it);
    }

    return result;
}

// Function to generate 2D Gray code
std::vector<std::vector<std::string>>
generate2DGrayCode(int k)
{
    // Assert that k is an even number
    assert(k % 2 == 0 && "k must be an even number for 2D Gray coding!");

    int halfK = k / 2;

    // Generate 1D Gray code for the vertical and horizontal directions
    std::vector<std::string> verticalGrayCode = generateGrayCode(halfK);
    std::vector<std::string> horizontalGrayCode = generateGrayCode(halfK);

    // Construct 2D Gray code matrix
    std::vector<std::vector<std::string>> grayCode2D;

    for (const auto &vertical : verticalGrayCode)
    {
        std::vector<std::string> row;
        for (const auto &horizontal : horizontalGrayCode)
        {
            row.push_back(vertical + horizontal);
        }
        grayCode2D.push_back(row);
    }

    return grayCode2D;
}

// Function to convert binary string to decimal
int
binaryToDecimal(const std::string &binary)
{
    // bitset use LSB first memory layout
    return std::bitset<64>(binary).to_ulong();
}

// Function to generate 2D Gray code in decimal representation
std::vector<std::vector<int>>
generate2DGrayCodeDecimal(int k)
{
    // Generate the 2D Gray code in binary
    std::vector<std::vector<std::string>> grayCode2D = generate2DGrayCode(k);

    // Convert binary Gray code to decimal
    std::vector<std::vector<int>> grayCode2DDecimal;

    for (const auto &row : grayCode2D)
    {
        std::vector<int> decimalRow;
        for (const auto &binary : row)
        {
            decimalRow.push_back(binaryToDecimal(binary));
        }
        grayCode2DDecimal.push_back(decimalRow);
    }

    return grayCode2DDecimal;
}

// Function to generate QAM symbols
std::vector<gr_complex>
generateQAMSymbols(int k, bool normalize_power)
{
    // Ensure k is even
    assert(k % 2 == 0 && "k must be an even number!");

    int M = 1 << k;          // Total number of points
    int m = 1 << (k / 2);    // Points along each axis

    // Generate 2D Gray code in decimal representation
    std::vector<std::vector<int>> grayCode2DDecimal = generate2DGrayCodeDecimal(k);

    // Initialize QAM symbol vector
    std::vector<gr_complex> qamSymbols(M);

    // Calculate QAM symbols based on position
    for (int r = 0; r < m; ++r)
    {
        for (int s = 0; s < m; ++s)
        {
            /// int decimalValue = grayCode2DDecimal[r][s];
            int decimalValue = grayCode2DDecimal[s][r];
            gr_complex symbol = {2.0f * s - (m - 1.0f), -2.0f * r + (m - 1.0f)};
            qamSymbols[decimalValue] = symbol;
        }
    }

    // Normalize power if required
    if (normalize_power)
    {
        float power = 0.0;
        for (const auto &symbol : qamSymbols)
        {
            power += std::norm(symbol); // Sum of absolute square values
        }
        power = power / M;
        power = std::sqrt(power); // Square root of average power
        for (auto &symbol : qamSymbols)
        {
            symbol /= power; // Normalize each symbol
        }
    }

    return qamSymbols;
}

/**
 * @brief Constructor for QAMModulator.
 * @param k Number of bits per symbol (must be even).
 * @param normalize_power Whether to normalize the average power of the QAM symbols to 1 (default is true).
 *
 * The constructor initializes the QAM modulator by generating the QAM symbols,
 * and calculating sorted real/imaginary parts and distances.
 *
 * @throws std::invalid_argument if k is not even.
 */
QAMModulator::QAMModulator(int k, bool normalize_power) : k(k)
{
    points = generateQAMSymbols(k, normalize_power);

    grayCode2DDecimal = generate2DGrayCodeDecimal(k);
    grayCode2D = generate2DGrayCode(k);

    // Extract real parts and sort uniquely
    std::vector<float> real_parts;
    for (const auto &symbol : points)
    {
        real_parts.push_back(symbol.real());
    }
    std::sort(real_parts.begin(), real_parts.end());
    real_parts.erase(std::unique(real_parts.begin(), real_parts.end()), real_parts.end());
    points_real_sorted = real_parts;

    // Extract imaginary parts and sort uniquely
    std::vector<float> imag_parts;
    for (const auto &symbol : points)
    {
        imag_parts.push_back(symbol.imag());
    }
    std::sort(imag_parts.begin(), imag_parts.end());
    imag_parts.erase(std::unique(imag_parts.begin(), imag_parts.end()), imag_parts.end());
    points_imag_sorted = imag_parts;

    // Calculate distances between consecutive points
    if (points_real_sorted.size() > 1)
    {
        distance_real = points_real_sorted[1] - points_real_sorted[0];
    }
    else
    {
        distance_real = 0.0;
    }

    if (points_imag_sorted.size() > 1)
    {
        distance_imag = points_imag_sorted[1] - points_imag_sorted[0];
    }
    else
    {
        distance_imag = 0.0;
    }
}


// Method to modulate a bit stream
/**
 * @brief Modulates a given bitstream into a stream of QAM symbols.
 * @param bit_stream The input binary bitstream. Its length must be an integer multiple of k.
 * @return A vector of complex numbers, where each number corresponds to a QAM symbol.
 *
 * The method splits the bitstream into chunks of k bits, converts each chunk into a decimal
 * index using the binaryToDecimal function, and maps the index to the corresponding QAM symbol.
 *
 * @throws std::invalid_argument if bit_stream length is not a multiple of k.
 */
std::vector<gr_complex>
QAMModulator::Modulate(const std::string &bit_stream)
{
    // Check that bit_stream length is an integer multiple of k
    if (bit_stream.size() % k != 0)
    {
        throw std::invalid_argument("Bit stream length must be a multiple of k!");
    }

    std::vector<gr_complex> symbol_stream;

    // Loop through the bit stream k bits at a time
    for (size_t i = 0; i < bit_stream.size(); i += k)
    {
        // Extract k bits as a substring
        std::string k_bits = bit_stream.substr(i, k);

        // Convert k bits to decimal (symbol index)
        int symbol_index = binaryToDecimal(k_bits);

        // Get the corresponding QAM symbol
        symbol_stream.push_back(points[symbol_index]);
    }

    return symbol_stream;
}

/**
 * @brief Demodulates an Eigen tensor of complex QAM symbols into a binary bit stream.
 * @param input_tensor The input Eigen tensor containing QAM symbols as complex numbers.
 * @return A binary string representing the demodulated bit stream.
 */
std::string
QAMModulator::Demodulate(const CTensor1D &input_tensor)
{
    // Get the number of symbols in the input tensor
    int dim0 = input_tensor.dimension(0);

    // Separate real and imaginary parts
    Tensor1D input_real = input_tensor.real();
    Tensor1D input_imag = input_tensor.imag();

    // Normalize by distances
    Tensor1D input_normalized_real = input_real / distance_real;
    Tensor1D input_normalized_imag = input_imag / distance_imag;

    // Floor the values
    input_normalized_real = input_normalized_real.floor();
    input_normalized_imag = input_normalized_imag.floor();

    // Limit the values to the valid range
    float max_value = std::pow(2, k / 2 - 1) - 1;
    float min_value = -std::pow(2, k / 2 - 1);

    // Clip values for real parts
    input_normalized_real = input_normalized_real.cwiseMin(max_value).cwiseMax(min_value);

    // Clip values for imaginary parts
    input_normalized_imag = input_normalized_imag.cwiseMin(max_value).cwiseMax(min_value);

    // Shift values to be zero-indexed
    input_normalized_real = input_normalized_real - min_value;
    input_normalized_imag = input_normalized_imag - min_value;

    // Convert normalized real and imaginary parts to indices
    Eigen::Tensor<int, 1> col_indices = input_normalized_real.cast<int>();
    Eigen::Tensor<int, 1> row_indices = input_normalized_imag.cast<int>();

    // Construct the final bit stream
    std::string final_string;
    for (int i = 0; i < dim0; i++)
    {
        // std::cout << "Integer mapped element (" << row_indices[i] << "," << col_indices[i] << ")" << std::endl ;
        /// final_string += grayCode2D[(std::pow(2, k/2)-1) - row_indices(i)][col_indices(i)];
        final_string += grayCode2D[col_indices(i)][(std::pow(2, k / 2) - 1) - row_indices(i)];
    }

    return final_string;
}

