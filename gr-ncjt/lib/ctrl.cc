/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ctrl.h"
#include "qam_constellation.h"
#include <cmath> // std::sqrt, std::fabs
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "common.h"

namespace gr
{
namespace ncjt
{

    // --------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------
    CTRL::CTRL(bool debug)
        : d_debug(debug),
          d_seq_number(0),
          d_reserved(0),
          d_nstrm_phase1(0),
          d_mod_type_phase1(0),
          d_coding_rate_phase1(0),
          d_nstrm_phase2(0),
          d_mod_type_phase2(0),
          d_coding_rate_phase2(0),
          d_nstrm_phase3(0),
          d_mod_type_phase3(0),
          d_coding_rate_phase3(0),
          d_data_checksum(0),
          d_ctrl_checksum(0),
          d_extended(0),
          d_polar_inited_64(false),
          d_polar_inited_128(false)
    {
        // For 128-bit block, we define a reliability sequence (least reliable to most reliable).
        // (This is just a demo example; in practice you'd use a standard 5G or known polar sequence.)
        d_rel_seq_128 = {
            33, 34, 35, 36, 37, 38, 40, 41, 42, 44, 48, 49, 76, 52, 56, 0,
            65, 66, 67, 68, 69, 70, 72, 73, 74, 50, 1, 2, 3, 4, 5, 6,
            7, 8, 9, 10, 11, 12, 32, 14, 16, 17, 18, 19, 20, 21, 22, 24,
            25, 26, 28, 13, 64, 112, 96, 80, 81, 82, 104, 84, 88, 97, 100, 98,
            15, 23, 27, 29, 39, 30, 43, 45, 46, 51, 71, 53, 75, 54, 77, 57,
            83, 78, 58, 85, 86, 60, 89, 99, 90, 101, 102, 92, 105, 106, 108, 113,
            114, 116, 120, 31, 47, 55, 79, 59, 87, 61, 91, 62, 103, 93, 107, 94,
            109, 115, 110, 117, 118, 121, 122, 124, 63, 95, 111, 119, 123, 125, 126, 127
        };

        // For 256-bit block, a reliability sequence of length 256.
        d_rel_seq_256 = {
            97, 96, 92, 90, 89, 88, 86, 85, 84, 83, 82, 81, 80, 142, 77, 76,
            75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 78, 141, 140, 139, 138,
            137, 136, 135, 134, 133, 132, 131, 130, 129, 98, 120, 116, 114, 113, 112, 108,
            106, 105, 104, 102, 101, 100, 99, 0, 27, 26, 25, 24, 23, 22, 21, 20,
            19, 18, 17, 16, 15, 64, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4,
            3, 2, 1, 14, 60, 58, 57, 56, 54, 53, 52, 51, 50, 49, 48, 46,
            45, 28, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 30, 29,
            44, 202, 160, 161, 162, 201, 200, 163, 164, 165, 166, 168, 169, 204, 198, 172,
            197, 196, 195, 194, 193, 176, 177, 178, 192, 180, 170, 148, 150, 147, 152, 153,
            154, 156, 228, 146, 145, 149, 225, 144, 224, 216, 184, 212, 210, 209, 208, 128,
            226, 232, 31, 47, 55, 240, 59, 79, 61, 87, 62, 91, 93, 103, 143, 94,
            107, 151, 109, 155, 115, 110, 167, 157, 117, 171, 158, 118, 173, 121, 179, 174,
            122, 199, 181, 203, 182, 124, 205, 185, 211, 206, 186, 213, 214, 188, 217, 227,
            218, 229, 230, 220, 233, 234, 236, 241, 63, 242, 95, 244, 111, 159, 119, 175,
            123, 248, 183, 125, 207, 187, 126, 215, 189, 219, 190, 231, 221, 235, 222, 237,
            243, 238, 245, 246, 249, 127, 250, 191, 252, 223, 239, 247, 251, 253, 254, 255
        };

        NCJT_LOG(d_debug, "CTRL object created.");
    }

    // --------------------------------------------------------------------
    // init_polar_64: for 64->128
    // --------------------------------------------------------------------
    void CTRL::init_polar_64()
    {
        if (d_polar_inited_64)
        {
            return;
        }
        d_polar_inited_64 = true;

        int block_size = 128;
        int num_info_bits = 64;
        int num_frozen_bits = block_size - num_info_bits; // 64

        std::vector<int> frozen_positions(num_frozen_bits);
        for (int i = 0; i < num_frozen_bits; i++)
        {
            frozen_positions[i] = d_rel_seq_128[i];
        }
        std::sort(frozen_positions.begin(), frozen_positions.end());
        std::vector<uint8_t> frozen_vals(num_frozen_bits, 0);

        d_polar_enc_64 = std::static_pointer_cast<gr::fec::code::polar_encoder>(
            gr::fec::code::polar_encoder::make(
                block_size,
                num_info_bits,
                frozen_positions,
                frozen_vals,
                false // unpacked bits
                ));

        d_polar_dec_64 = std::static_pointer_cast<gr::fec::code::polar_decoder_sc>(
            gr::fec::code::polar_decoder_sc::make(
                block_size,
                num_info_bits,
                frozen_positions,
                frozen_vals));

        NCJT_LOG(d_debug, "Polar encoder/decoder (64->128) initialized.");
    }

    // --------------------------------------------------------------------
    // init_polar_128: for 128->256
    // --------------------------------------------------------------------
    void CTRL::init_polar_128()
    {
        if (d_polar_inited_128)
        {
            return;
        }
        d_polar_inited_128 = true;

        int block_size = 256;
        int num_info_bits = 128;
        int num_frozen_bits = block_size - num_info_bits; // 128

        std::vector<int> frozen_positions(num_frozen_bits);
        for (int i = 0; i < num_frozen_bits; i++)
        {
            frozen_positions[i] = d_rel_seq_256[i];
        }
        std::sort(frozen_positions.begin(), frozen_positions.end());
        std::vector<uint8_t> frozen_vals(num_frozen_bits, 0);

        d_polar_enc_128 = std::static_pointer_cast<gr::fec::code::polar_encoder>(
            gr::fec::code::polar_encoder::make(
                block_size,
                num_info_bits,
                frozen_positions,
                frozen_vals,
                false));

        d_polar_dec_128 = std::static_pointer_cast<gr::fec::code::polar_decoder_sc>(
            gr::fec::code::polar_decoder_sc::make(
                block_size,
                num_info_bits,
                frozen_positions,
                frozen_vals));

        NCJT_LOG(d_debug, "Polar encoder/decoder (128->256) initialized.");
    }

    // --------------------------------------------------------------------
    // QPSK: 64 bits => CRC10 => 128 bits => 64 QPSK
    // --------------------------------------------------------------------
    std::vector<gr_complex> CTRL::pack_and_modulate_qpsk()
    {
        // 1) pack 64 bits + compute CRC10 over [0..53]
        uint64_t cword = pack_64bits_and_crc();

        NCJT_LOG(d_debug, "\n\tTX cword = 0x"
                      << std::hex << cword << std::dec);

        // 2) build bits_64
        std::vector<uint8_t> bits_64(64);
        for (int i = 0; i < 64; i++)
        {
            bits_64[i] = (cword >> i) & 1ULL;
        }
        if (d_debug)
        {
            std::ostringstream oss;
            oss << "  64 TX info bits (LSB->MSB): ";
            for (int i = 0; i < 64; i++)
                oss << (int)bits_64[i];
            NCJT_LOG(d_debug, oss.str());
        }


        // 3) polar encode
        init_polar_64();
        std::vector<uint8_t> coded(128, 0);
        d_polar_enc_64->generic_work(bits_64.data(), coded.data());

        if (d_debug)
        {
            std::ostringstream oss;
            oss << "  128 TX coded bits (LSB->MSB): ";
            for (int i = 0; i < 128; i++)
                oss << (int)coded[i];
            NCJT_LOG(d_debug, oss.str());
        }

        // 4) map => 64 QPSK
        std::vector<gr_complex> syms(64);
        for (int i = 0; i < 64; i++)
        {
            int b0 = coded[2 * i + 0] & 1;
            int b1 = coded[2 * i + 1] & 1;
            int idx = (b1 << 1) | b0; // [0..3]
            syms[i] = CONST_QPSK[idx];
        }
        return syms;
    }

    bool CTRL::demodulate_and_unpack_qpsk(const std::vector<gr_complex> &syms,
                                          const std::vector<gr_complex> &csi)
    {
        if (syms.size() < 64)
        {
            NCJT_LOG(d_debug, "Not enough symbols for QPSK demodulation: "
                      << syms.size() << " < 64");
            return false;
        }

        // 1) Hard-decision demap for QPSK
        std::vector<float> llr(128, 0.0f);
        std::vector<uint8_t> detected_bits(128, 0);
        for (int i = 0; i < 64; i++)
        {
            // demap_symbol() is from common.h
            uint8_t bits[8]; // up to 8 bits, but we only need 2
            demap_symbol(syms[i].real(), syms[i].imag(), csi[i].real(), 2, bits);
            for (int b = 0; b < 2; b++)
            {
                detected_bits[2 * i + b] = bits[b];
                llr[2 * i + b] = (bits[b] == 0) ? -10.0f : +10.0f;
            }
        }

        if (d_debug)
        {
            std::ostringstream oss;
            oss << " Detected bits:";
            for (int i = 0; i < 128; i++)
                oss << (int)detected_bits[i];
            NCJT_LOG(d_debug, oss.str());
        }

        // 2) polar decode => bits_64
        init_polar_64();
        std::vector<uint8_t> bits_64(64, 0);
        d_polar_dec_64->generic_work(llr.data(), bits_64.data());

        // 3) re-form the 64-bit word
        uint64_t cword = 0ULL;
        for (int i = 0; i < 64; i++)
        {
            cword |= (uint64_t(bits_64[i]) & 1ULL) << i;
        }

        if (d_debug)
        {
            std::ostringstream oss;
            oss << "  64 RX info bits (LSB->MSB): ";
            for (int i = 0; i < 64; i++)
            {
                oss << ((cword >> i) & 1ULL);
            }
            NCJT_LOG(d_debug, oss.str());
        }

        // 4) unpack => check CRC
        bool ok = unpack_64bits_and_crc(cword);
        if (!ok)
        {
            NCJT_LOG(d_debug, "CRC mismatch!");
        }

        // In QPSK mode, extended is ignored
        d_extended = 0ULL;
        return ok;
    }

    // --------------------------------------------------------------------
    // 16QAM: 128 bits => 256 bits => 64 16QAM
    // --------------------------------------------------------------------
    std::vector<gr_complex> CTRL::pack_and_modulate_16qam()
    {
        // 1) form ctrl64, ext64 and compute the correct 10-bit CRC
        uint64_t ctrl64 = 0ULL, ext64 = d_extended;
        pack_128bits_and_crc(ctrl64, ext64);

        if (d_debug)
        {
            std::ostringstream oss;
            oss << "TX ctrl64=0x" << std::hex << ctrl64
                << " ext64=0x" << ext64 << std::dec;
            NCJT_LOG(d_debug, oss.str());
        }

        // 2) build bits_128
        std::vector<uint8_t> bits_128(128, 0);
        for (int i = 0; i < 64; i++)
        {
            bits_128[i] = (ctrl64 >> i) & 1;
            bits_128[64 + i] = (ext64 >> i) & 1;
        }

        if (d_debug)
        {
            std::ostringstream oss;
            oss << "  TX bits_128[0..7]: ";
            for (int i = 0; i < 8; i++)
                oss << (int)bits_128[i];
            oss << " ...";
            NCJT_LOG(d_debug, oss.str());
        }

        // 3) polar encode => 256 bits
        init_polar_128();
        std::vector<uint8_t> coded(256, 0);
        d_polar_enc_128->generic_work(bits_128.data(), coded.data());

        // 4) map => 64 of 16QAM
        std::vector<gr_complex> syms(64);
        for (int i = 0; i < 64; i++)
        {
            int b0 = coded[4 * i + 0] & 1;
            int b1 = coded[4 * i + 1] & 1;
            int b2 = coded[4 * i + 2] & 1;
            int b3 = coded[4 * i + 3] & 1;
            int idx = (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
            syms[i] = CONST_16QAM[idx];
        }
        return syms;
    }

    bool CTRL::demodulate_and_unpack_16qam(const std::vector<gr_complex> &syms,
                                           const std::vector<gr_complex> &csi)
    {
        if (syms.size() < 64)
        {
            NCJT_LOG(d_debug, "Not enough symbols for 16QAM demodulation: "
                      << syms.size() << " < 64");
            return false;
        }

        // 1) Hard-decision demap for 16QAM, same approach as QPSK
        //    (We produce bits, then set LLR = ±10.)
        std::vector<uint8_t> detected_bits(256, 0);
        std::vector<float> llr(256, 0.0f);

        for (int i = 0; i < 64; i++)
        {
            uint8_t bits[8]; // up to 8 bits, but we only need 4 for 16QAM
            demap_symbol(syms[i].real(), syms[i].imag(), csi[i].real(), 4, bits);

            for (int b = 0; b < 4; b++)
            {
                detected_bits[4 * i + b] = bits[b];
                llr[4 * i + b] = (bits[b] == 1) ? +10.0f : -10.0f;
            }
        }

        if (d_debug)
        {
            std::ostringstream oss;
            oss << "First 128 LLRs: ";
            for (int i = 0; i < 128; i++)
                oss << llr[i] << " ";
            oss << "...";
            NCJT_LOG(d_debug, oss.str());
        }

        // 2) decode => 128 bits
        init_polar_128();
        std::vector<uint8_t> bits_128(128, 0);
        d_polar_dec_128->generic_work(llr.data(), bits_128.data());

        // 3) reconstruct ctrl64, ext64
        uint64_t ctrl64 = 0ULL, ext64 = 0ULL;
        for (int i = 0; i < 64; i++)
        {
            ctrl64 |= (uint64_t(bits_128[i]) & 1ULL) << i;
            ext64 |= (uint64_t(bits_128[64 + i]) & 1ULL) << i;
        }

        NCJT_LOG(d_debug, "RX ctrl64=0x" << std::hex << ctrl64
                      << " ext64=0x" << ext64 << std::dec);

        // 4) check CRC
        bool ok = unpack_128bits_and_crc(ctrl64, ext64);
        if (!ok)
        {
            d_extended = 0ULL;

            NCJT_LOG(d_debug, "CRC mismatch!");
            return false;
        }
        return true;
    }

    // --------------------------------------------------------------------
    // 64-bit pack/unpack w/ CRC (QPSK path)
    // --------------------------------------------------------------------
    uint64_t CTRL::pack_64bits_and_crc()
    {
        // Build the 64-bit control word, ignoring bits [54..63] initially
        uint64_t w = 0ULL;
        w |= ((uint64_t)d_seq_number & 0xFFFF) << 0;
        w |= ((uint64_t)d_reserved & 0x0F) << 16;
        w |= ((uint64_t)d_nstrm_phase1 & 0x01) << 20;
        w |= ((uint64_t)d_mod_type_phase1 & 0x03) << 21;
        w |= ((uint64_t)d_coding_rate_phase1 & 0x07) << 23;
        w |= ((uint64_t)d_nstrm_phase2 & 0x01) << 26;
        w |= ((uint64_t)d_mod_type_phase2 & 0x03) << 27;
        w |= ((uint64_t)d_coding_rate_phase2 & 0x07) << 29;
        w |= ((uint64_t)d_nstrm_phase3 & 0x01) << 32;
        w |= ((uint64_t)d_mod_type_phase3 & 0x03) << 33;
        w |= ((uint64_t)d_coding_rate_phase3 & 0x07) << 35;
        w |= ((uint64_t)d_data_checksum & 0xFFFF) << 38; // bits [38..53]

        // compute CRC10 over bits [0..53]
        std::vector<uint8_t> bits_54(54, 0);
        for (int i = 0; i < 54; i++)
        {
            bits_54[i] = (w >> i) & 1;
        }
        uint16_t c10 = compute_crc10(bits_54);

        // place c10 into bits [54..63]
        NCJT_LOG(d_debug, "CRC10 = " << std::hex << c10 << std::dec);
        return w | ((uint64_t)c10 << 54);
    }

    bool CTRL::unpack_64bits_and_crc(uint64_t word)
    {
        uint16_t stored_crc = (word >> 54) & 0x3FF;
        uint64_t data_wo_crc = word & 0x3FFFFFFFFFFFFFULL; // bits [0..53] only

        // recompute
        std::vector<uint8_t> bits_54(54, 0);
        for (int i = 0; i < 54; i++)
        {
            bits_54[i] = (data_wo_crc >> i) & 1;
        }
        uint16_t calc = compute_crc10(bits_54);
        NCJT_LOG(d_debug, "Calculated CRC10 = " << std::hex << calc
                    << " Stored CRC10 = " << stored_crc << std::dec);
        if (calc != stored_crc)
        {
            return false;
        }
        d_ctrl_checksum = stored_crc;

        // parse
        d_seq_number = (uint16_t)((data_wo_crc >> 0) & 0xFFFF);
        d_reserved = (uint8_t)((data_wo_crc >> 16) & 0x0F);
        d_nstrm_phase1 = (uint8_t)((data_wo_crc >> 20) & 0x01);
        d_mod_type_phase1 = (uint8_t)((data_wo_crc >> 21) & 0x03);
        d_coding_rate_phase1 = (uint8_t)((data_wo_crc >> 23) & 0x07);
        d_nstrm_phase2 = (uint8_t)((data_wo_crc >> 26) & 0x01);
        d_mod_type_phase2 = (uint8_t)((data_wo_crc >> 27) & 0x03);
        d_coding_rate_phase2 = (uint8_t)((data_wo_crc >> 29) & 0x07);
        d_nstrm_phase3 = (uint8_t)((data_wo_crc >> 32) & 0x01);
        d_mod_type_phase3 = (uint8_t)((data_wo_crc >> 33) & 0x03);
        d_coding_rate_phase3 = (uint8_t)((data_wo_crc >> 35) & 0x07);
        d_data_checksum = (uint16_t)((data_wo_crc >> 38) & 0xFFFF);

        return true;
    }

    // --------------------------------------------------------------------
    // 128-bit pack/unpack w/ CRC (16QAM extended path)
    // --------------------------------------------------------------------
    void CTRL::pack_128bits_and_crc(uint64_t &ctrl64, uint64_t &ext64)
    {
        // 1) Build ctrl64 w/o final CRC in bits [54..63]
        ctrl64 = 0ULL;
        ctrl64 |= ((uint64_t)d_seq_number & 0xFFFF) << 0;
        ctrl64 |= ((uint64_t)d_reserved & 0x0F) << 16;
        ctrl64 |= ((uint64_t)d_nstrm_phase1 & 0x01) << 20;
        ctrl64 |= ((uint64_t)d_mod_type_phase1 & 0x03) << 21;
        ctrl64 |= ((uint64_t)d_coding_rate_phase1 & 0x07) << 23;
        ctrl64 |= ((uint64_t)d_nstrm_phase2 & 0x01) << 26;
        ctrl64 |= ((uint64_t)d_mod_type_phase2 & 0x03) << 27;
        ctrl64 |= ((uint64_t)d_coding_rate_phase2 & 0x07) << 29;
        ctrl64 |= ((uint64_t)d_nstrm_phase3 & 0x01) << 32;
        ctrl64 |= ((uint64_t)d_mod_type_phase3 & 0x03) << 33;
        ctrl64 |= ((uint64_t)d_coding_rate_phase3 & 0x07) << 35;
        ctrl64 |= ((uint64_t)d_data_checksum & 0xFFFF) << 38;

        // Make sure bits [54..63] are zeroed before computing extended CRC
        ctrl64 &= ~(0x3FFULL << 54);

        ext64 = d_extended;

        // 2) Build a 128-bit array from ctrl64 + ext64
        std::vector<uint8_t> bits_128(128, 0);
        for (int i = 0; i < 64; i++)
        {
            bits_128[i] = (ctrl64 >> i) & 1;
            bits_128[64 + i] = (ext64 >> i) & 1;
        }

        // 3) Compute CRC10 over only the first 54 bits of ctrl64 + all 64 bits of ext64
        //    i.e. total 118 bits => [0..53] + [64..127].
        std::vector<uint8_t> relevant(118, 0);
        for (int i = 0; i < 54; i++)
        {
            relevant[i] = bits_128[i]; // bits [0..53] of ctrl64
        }
        for (int i = 0; i < 64; i++)
        {
            relevant[54 + i] = bits_128[64 + i]; // all 64 extended bits
        }

        uint16_t c10 = compute_crc10(relevant);

        // 4) Place c10 in bits [54..63] of ctrl64
        ctrl64 |= ((uint64_t)c10 << 54);
    }

    bool CTRL::unpack_128bits_and_crc(uint64_t &ctrl64, uint64_t &ext64)
    {
        // 1) Read stored CRC from bits [54..63]
        uint16_t stored_crc = (ctrl64 >> 54) & 0x3FF;

        // 2) Rebuild the 128 bits from ctrl64 + ext64
        std::vector<uint8_t> bits_128(128, 0);
        for (int i = 0; i < 64; i++)
        {
            bits_128[i] = (ctrl64 >> i) & 1;
            bits_128[64 + i] = (ext64 >> i) & 1;
        }

        // 3) Compute CRC10 over bits [0..53] plus [64..127], ignoring [54..63]
        std::vector<uint8_t> relevant(118, 0);
        for (int i = 0; i < 54; i++)
        {
            relevant[i] = bits_128[i];
        }
        for (int i = 0; i < 64; i++)
        {
            relevant[54 + i] = bits_128[64 + i];
        }

        uint16_t calc = compute_crc10(relevant);
        if (calc != stored_crc)
        {
            return false;
        }
        d_ctrl_checksum = stored_crc;

        // 4) Parse fields from ctrl64 (minus the CRC bits [54..63])
        uint64_t data_wo_crc = ctrl64 & 0x3FFFFFFFFFFFFFULL;
        d_seq_number = (uint16_t)((data_wo_crc >> 0) & 0xFFFF);
        d_reserved = (uint8_t)((data_wo_crc >> 16) & 0x0F);
        d_nstrm_phase1 = (uint8_t)((data_wo_crc >> 20) & 0x01);
        d_mod_type_phase1 = (uint8_t)((data_wo_crc >> 21) & 0x03);
        d_coding_rate_phase1 = (uint8_t)((data_wo_crc >> 23) & 0x07);
        d_nstrm_phase2 = (uint8_t)((data_wo_crc >> 26) & 0x01);
        d_mod_type_phase2 = (uint8_t)((data_wo_crc >> 27) & 0x03);
        d_coding_rate_phase2 = (uint8_t)((data_wo_crc >> 29) & 0x07);
        d_nstrm_phase3 = (uint8_t)((data_wo_crc >> 32) & 0x01);
        d_mod_type_phase3 = (uint8_t)((data_wo_crc >> 33) & 0x03);
        d_coding_rate_phase3 = (uint8_t)((data_wo_crc >> 35) & 0x07);
        d_data_checksum = (uint16_t)((data_wo_crc >> 38) & 0xFFFF);

        d_extended = ext64;
        return true;
    }

    // --------------------------------------------------------------------
    // Compute CRC10
    // --------------------------------------------------------------------
    uint16_t CTRL::compute_crc10(const std::vector<uint8_t> &bits) const
    {
        // ATM CRC10 polynomial: x^10 + x^3 + 1 (0x233)
        uint16_t crc = 0;
        for (size_t i = 0; i < bits.size(); i++)
        {
            // Note: We process from the *end* of the vector to front
            bool in_bit = (bits[bits.size() - 1 - i] != 0);
            bool top_bit = ((crc & 0x200) != 0);
            bool feedback = (in_bit ^ top_bit);
            crc <<= 1;
            crc &= 0x3FF;
            if (feedback)
            {
                crc ^= 0x233;
            }
        }
        return (crc & 0x3FF);
    }

    // --------------------------------------------------------------------
    // Simple field getters/setters
    // --------------------------------------------------------------------
    void CTRL::set_seq_number(uint16_t seq) { d_seq_number = seq; }
    uint16_t CTRL::get_seq_number() const { return d_seq_number; }

    void CTRL::set_reserved(uint8_t r) { d_reserved = r & 0x0F; }
    uint8_t CTRL::get_reserved() const { return d_reserved; }

    void CTRL::set_nstrm_phase1(uint8_t v) { d_nstrm_phase1 = v & 0x01; }
    uint8_t CTRL::get_nstrm_phase1() const { return d_nstrm_phase1; }

    void CTRL::set_mod_type_phase1(uint8_t v) { d_mod_type_phase1 = v & 0x03; }
    uint8_t CTRL::get_mod_type_phase1() const { return d_mod_type_phase1; }

    void CTRL::set_coding_rate_phase1(uint8_t v) { d_coding_rate_phase1 = v & 0x07; }
    uint8_t CTRL::get_coding_rate_phase1() const { return d_coding_rate_phase1; }

    void CTRL::set_nstrm_phase2(uint8_t v) { d_nstrm_phase2 = v & 0x01; }
    uint8_t CTRL::get_nstrm_phase2() const { return d_nstrm_phase2; }

    void CTRL::set_mod_type_phase2(uint8_t v) { d_mod_type_phase2 = v & 0x03; }
    uint8_t CTRL::get_mod_type_phase2() const { return d_mod_type_phase2; }

    void CTRL::set_coding_rate_phase2(uint8_t v) { d_coding_rate_phase2 = v & 0x07; }
    uint8_t CTRL::get_coding_rate_phase2() const { return d_coding_rate_phase2; }

    void CTRL::set_nstrm_phase3(uint8_t v) { d_nstrm_phase3 = v & 0x01; }
    uint8_t CTRL::get_nstrm_phase3() const { return d_nstrm_phase3; }

    void CTRL::set_mod_type_phase3(uint8_t v) { d_mod_type_phase3 = v & 0x03; }
    uint8_t CTRL::get_mod_type_phase3() const { return d_mod_type_phase3; }

    void CTRL::set_coding_rate_phase3(uint8_t v) { d_coding_rate_phase3 = v & 0x07; }
    uint8_t CTRL::get_coding_rate_phase3() const { return d_coding_rate_phase3; }

    void CTRL::set_data_checksum(uint16_t c) { d_data_checksum = c; }
    uint16_t CTRL::get_data_checksum() const { return d_data_checksum; }

    uint16_t CTRL::get_ctrl_checksum() const { return d_ctrl_checksum; }

    void CTRL::set_extended(uint64_t ext) { d_extended = ext; }
    uint64_t CTRL::get_extended() const { return d_extended; }

    void CTRL::set_raw(unsigned __int128 value)
    {
        uint64_t local_ctrl64 = (uint64_t)(value & 0xFFFFFFFFFFFFFFFFULL);
        uint64_t local_ext64  = (uint64_t)(value >> 64);
        unpack_128bits_and_crc(local_ctrl64, local_ext64);
    }

    unsigned __int128 CTRL::get_raw() const
    {
        uint64_t local_ctrl64, local_ext64;
        // pack_128bits_and_crc modifies CRC bits, so use a non-const pointer
        CTRL* self = const_cast<CTRL*>(this);
        self->pack_128bits_and_crc(local_ctrl64, local_ext64);

        unsigned __int128 result = local_ext64;
        result <<= 64;
        result |= local_ctrl64;
        return result;
    }

} // namespace ncjt
} // namespace gr
