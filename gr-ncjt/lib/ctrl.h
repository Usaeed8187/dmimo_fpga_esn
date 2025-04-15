/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_CTRL_H
#define INCLUDED_NCJT_CTRL_H

#include <gnuradio/gr_complex.h>
#include <vector>
#include <cstdint>

// We need these FEC headers for polar coding
#include <gnuradio/fec/polar_encoder.h>
#include <gnuradio/fec/polar_decoder_sc.h>

namespace gr {
namespace ncjt {

/*!
 * \brief CTRL class that packs/unpacks control fields into 64 bits,
 *        plus a 64-bit extended field. Then does rate-1/2 Polar coding
 *        and QPSK or 16QAM mapping.
 *
 * In QPSK mode:
 *   - Only the first 64 bits are used (the extended field is ignored).
 *   - 64 bits -> 128 coded bits -> 64 QPSK symbols.
 *
 * In 16QAM mode:
 *   - All 128 bits (64 control + 64 extended) are used.
 *   - The 10-bit CRC is computed over the entire 128 bits,
 *     and stored in bits [54..63] of the control word.
 *   - 128 bits -> 256 coded bits -> 64 16QAM symbols.
 *
 * Bit Layout of the 64-bit control word:
 *    [0..15]   seq_number
 *    [16..19]  reserved (4 bits)
 *    [20]      nstrm_phase1 (1 bit)
 *    [21..22]  mod_type_phase1 (2 bits)
 *    [23..25]  coding_rate_phase1 (3 bits)
 *    [26]      nstrm_phase2
 *    [27..28]  mod_type_phase2
 *    [29..31]  coding_rate_phase2
 *    [32]      nstrm_phase3
 *    [33..34]  mod_type_phase3
 *    [35..37]  coding_rate_phase3
 *    [38..53]  data_checksum (16 bits)
 *    [54..63]  ctrl_checksum (10 bits used)
 *
 * Additional 64-bit "extended" field used only in 16QAM mode.
 */
class CTRL
{
public:
    /*!
     * \brief The only constructor.
     * \param debug enable debug prints
     */
    CTRL(bool debug = false);

    // -------------------------
    // Detailed field accessors
    // -------------------------
    void set_seq_number(uint16_t seq);
    uint16_t get_seq_number() const;

    void set_reserved(uint8_t r);
    uint8_t get_reserved() const;

    void set_nstrm_phase1(uint8_t v);
    uint8_t get_nstrm_phase1() const;

    void set_mod_type_phase1(uint8_t v);
    uint8_t get_mod_type_phase1() const;

    void set_coding_rate_phase1(uint8_t v);
    uint8_t get_coding_rate_phase1() const;

    void set_nstrm_phase2(uint8_t v);
    uint8_t get_nstrm_phase2() const;

    void set_mod_type_phase2(uint8_t v);
    uint8_t get_mod_type_phase2() const;

    void set_coding_rate_phase2(uint8_t v);
    uint8_t get_coding_rate_phase2() const;

    void set_nstrm_phase3(uint8_t v);
    uint8_t get_nstrm_phase3() const;

    void set_mod_type_phase3(uint8_t v);
    uint8_t get_mod_type_phase3() const;

    void set_coding_rate_phase3(uint8_t v);
    uint8_t get_coding_rate_phase3() const;

    void set_data_checksum(uint16_t c);
    uint16_t get_data_checksum() const;

    uint16_t get_ctrl_checksum() const; // read-only

    void set_extended(uint64_t ext);
    uint64_t get_extended() const;

    // Return the number of symbols for each scheme
    int num_symbols_qpsk() const { return 64; }
    int num_symbols_16qam() const { return 64; }

    // QPSK path => 64 symbols
    std::vector<gr_complex> pack_and_modulate_qpsk();
 bool demodulate_and_unpack_qpsk(const std::vector<gr_complex> &syms,
                                 const std::vector<gr_complex> &csi);

    // 16QAM path => 64 symbols
    std::vector<gr_complex> pack_and_modulate_16qam();
    bool demodulate_and_unpack_16qam(const std::vector<gr_complex> &syms,
                                     const std::vector<gr_complex> &csi);

    void set_raw(unsigned __int128 val);
    unsigned __int128 get_raw() const;

private:
    bool d_debug;

    // New multi-phase bit layout fields
    uint16_t d_seq_number;        // bits [0..15]
    uint8_t d_reserved;           // bits [16..19]
    uint8_t d_nstrm_phase1;       // bit 20
    uint8_t d_mod_type_phase1;    // bits [21..22]
    uint8_t d_coding_rate_phase1; // bits [23..25]
    uint8_t d_nstrm_phase2;       // bit 26
    uint8_t d_mod_type_phase2;    // bits [27..28]
    uint8_t d_coding_rate_phase2; // bits [29..31]
    uint8_t d_nstrm_phase3;       // bit 32
    uint8_t d_mod_type_phase3;    // bits [33..34]
    uint8_t d_coding_rate_phase3; // bits [35..37]
    uint16_t d_data_checksum;     // bits [38..53]
    uint16_t d_ctrl_checksum;     // bits [54..63] (10 bits used)
    uint64_t d_extended;          // additional 64 bits

    // For polar coding (rate-1/2 always)
    bool d_polar_inited_64;   // for 64->128
    bool d_polar_inited_128;  // for 128->256
    std::shared_ptr<gr::fec::code::polar_encoder> d_polar_enc_64;
    std::shared_ptr<gr::fec::code::polar_decoder_sc> d_polar_dec_64;
    std::shared_ptr<gr::fec::code::polar_encoder> d_polar_enc_128;
    std::shared_ptr<gr::fec::code::polar_decoder_sc> d_polar_dec_128;

    // Reliability sequences (from least reliable to most reliable)
    // We'll take the first K as frozen bits, then the remaining are info bits.
    std::vector<int> d_rel_seq_128; // length=128
    std::vector<int> d_rel_seq_256; // length=256

    // Internal helpers
    void init_polar_64();   // 64 -> 128
    void init_polar_128();  // 128 -> 256

    // QPSK (64-bit) packing/unpacking w/ CRC10
    uint64_t pack_64bits_and_crc();
    bool unpack_64bits_and_crc(uint64_t word);

    // 16QAM (128-bit) packing/unpacking w/ CRC10
    void pack_128bits_and_crc(uint64_t &ctrl64, uint64_t &ext64);
    bool unpack_128bits_and_crc(uint64_t &ctrl64, uint64_t &ext64);

    // Bitwise CRC10/ATM
    uint16_t compute_crc10(const std::vector<uint8_t> &bits) const;
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_CTRL_H */
