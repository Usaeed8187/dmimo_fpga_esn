#include <iostream>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <cassert>
#include <iomanip>

// Include the HLL library and the QAM constellation definitions:
#include "hll/hll.h"
#include "qam_constellation.h"

// We'll use these from the gr::ncjt namespace (as in pdc_impl.cc)
using namespace gr::ncjt;

// Global QAM constellation references to be used both in main and demap_symbol:
const std::vector<gr_complex> *d_constellation_qpsk;
const std::vector<gr_complex> *d_constellation_16qam;
const std::vector<gr_complex> *d_constellation_64qam;
const std::vector<gr_complex> *d_constellation_256qam;

gr_complex demap_symbol(float x, float y, float csi_val,
                        int rx_modtype, uint8_t *outbits)
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
        outsym = (*d_constellation_qpsk)[(outbits[1] << 1) | outbits[0]];
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
        outsym = (*d_constellation_16qam)[outbits[3] << 3 | outbits[2] << 2 |
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
        outsym = (*d_constellation_64qam)[outbits[5] << 5 | outbits[4] << 4 |
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
        outsym = (*d_constellation_256qam)[outbits[7] << 7 | outbits[6] << 6 |
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

int main()
{
    int modtype = 6;                // 16QAM
    bool little_endian = true;      // true: little-endian, false: big-endian
    int num_symbols = 1 << modtype; // total symbols, e.g., 16 for modtype 4

    d_constellation_qpsk = &CONST_QPSK;
    d_constellation_16qam = &CONST_16QAM;
    d_constellation_64qam = &CONST_64QAM;
    d_constellation_256qam = &CONST_256QAM;

    // Replace the fixed-size vector with an empty vector with reserved capacity.
    std::vector<uint8_t> tx_bits;
    tx_bits.reserve(modtype * num_symbols);

    // Fill tx_bits with bit combinations based on modtype and endianness
    for (int i = 0; i < num_symbols; i++)
    {
        if (little_endian)
        {
            for (int b = 0; b < modtype; b++)
            {
                tx_bits.push_back((i >> b) & 1);
            }
        }
        else
        {
            for (int b = 0; b < modtype; b++)
            {
                tx_bits.push_back((i >> (modtype - 1 - b)) & 1);
            }
        }
    }

    std::vector<gr_complex> tx_syms(num_symbols);
    for (int k = 0; k < num_symbols; k++)
    {
        int offset = k * modtype;
        int val = 0;
        // gather bits for this symbol
        for (int j = 0; j < modtype; j++)
        {
            int bit_index = offset + j;
            val |= (tx_bits[bit_index] << j);
        }
        // map val to a constellation point
        gr_complex sym;
        switch (modtype)
        {
        case 2:
            sym = (*d_constellation_qpsk)[val];
            break;
        case 4:
            sym = (*d_constellation_16qam)[val];
            break;
        case 6:
            sym = (*d_constellation_64qam)[val];
            break;
        case 8:
            sym = (*d_constellation_256qam)[val];
            break;
        default:
            throw std::runtime_error("Invalid modulation type");
        }
        tx_syms[k] = sym;
    }

    std::vector<uint8_t> rx_bits;
    rx_bits.resize(modtype * num_symbols);
    std::vector<gr_complex> remapped_rx_syms(num_symbols);
    for (int k = 0; k < num_symbols; k++)
    {
        float x = tx_syms[k].real();
        float y = tx_syms[k].imag();
        uint8_t outbits[8] = {0};
        auto outsym = demap_symbol(x, y, 1.0f, modtype, outbits);
        for (int b = 0; b < modtype; b++)
        {
            rx_bits[k * modtype + b] = outbits[b];
        }
        remapped_rx_syms[k] = outsym;
    }
    // HLL
    std::unordered_map<int, std::unordered_map<int, HardLogLikelihoodVanilla *>> d_hlls;

    int num_copies = 1;
    int copies_count = num_copies;
    for (int mtype = 2; mtype <= 8; mtype += 2)
    {
        std::vector<gr_complex> constellation;
        switch (mtype)
        {
        case 2:
            constellation = CONST_QPSK;
            break;
        case 4:
            constellation = CONST_16QAM;
            break;
        case 6:
            constellation = CONST_64QAM;
            break;
        case 8:
            constellation = CONST_256QAM;
            break;
        }
        for (int copies = 1; copies <= num_copies; copies++)
        {
            d_hlls[mtype][copies] = new HardLogLikelihoodVanilla(
                constellation.data(),
                constellation.size(),
                mtype,
                true,
                true);
        }
    }

    //
    std::vector<float> rx_real(num_symbols * copies_count);
    std::vector<float> rx_imag(num_symbols * copies_count);
    std::vector<float> SNRs(num_symbols * copies_count);
    std::vector<float> llrs_hll;
    std::vector<uint8_t> bits_hll(num_symbols * modtype);
    // Fill those arrays
    int copy_id = 0;
    for (int q = 0; q < copies_count; q++)
    {
        auto snr_db = 10.0f;
        float snr_linear = std::pow(10.0f, snr_db / 10.0f);
        for (int i = 0; i < num_symbols; i++)
        {
            int idx = i * copies_count + copy_id;
            rx_real[idx] = tx_syms[i].real();
            rx_imag[idx] = tx_syms[i].imag();
            SNRs[idx] = snr_linear;
        }
        copy_id++;
    }
    d_hlls[modtype][copies_count]->Compute(rx_real.data(),
                                           rx_imag.data(),
                                           SNRs.data(),
                                           num_symbols,
                                           copies_count,
                                           llrs_hll,
                                           1);
    // Reverse the order of the LLRs
    for (size_t d = 0; d < llrs_hll.size(); d += modtype)
    {
        std::reverse(llrs_hll.begin() + d, llrs_hll.begin() + d + modtype);
    }
    for (int i=0; i<num_symbols*modtype; i++){
        bits_hll[i] = (llrs_hll[i] >= 0.0f) ? 1 : 0;
    }
    // Print
    for (int i = 0; i < num_symbols; i++)
    {
        // Print tx_bits in symbol
        std::cout << "[";
        for (int b = 0; b < modtype; b++)
        {
            std::cout << (int)tx_bits[i * modtype + b];
        }
        std::cout << "] -> ";
        // Print tx_syms with sign for positives
        std::cout << "[" << std::setw(5) << std::fixed << std::setprecision(2) << std::showpos
                  << tx_syms[i].real() << ", "
                  << std::setw(6) << std::fixed << std::setprecision(2) << tx_syms[i].imag() << std::noshowpos
                  << "] -> ";
        // Print rx_bits in symbol
        std::cout << "[";
        for (int b = 0; b < modtype; b++)
        {
            std::cout << (int)rx_bits[i * modtype + b];
        }
        std::cout << "] -> ";
        // Print remapped_rx_syms
        std::cout << "[" << std::setw(5) << std::fixed << std::setprecision(2) << std::showpos
                  << remapped_rx_syms[i].real() << ", "
                  << std::setw(6) << std::fixed << std::setprecision(2) << remapped_rx_syms[i].imag() << std::noshowpos
                  << "] -> ";
        // Print llrs_hll
        std::cout << "[";
        for (int b = 0; b < modtype; b++)
        {
            std::cout << std::setw(5) << std::fixed << std::setprecision(2) << llrs_hll[i * modtype + b] << ", ";
        }
        std::cout << "] -> ";
        // Print bits_hll
        std::cout << "[";
        for (int b = 0; b < modtype; b++)
        {
            std::cout << (int)bits_hll[i * modtype + b] << ", ";
        }
        std::cout << "] -> ";
        std::cout << std::endl;
    }

    std::cout << std::endl;
    return 0;
}
