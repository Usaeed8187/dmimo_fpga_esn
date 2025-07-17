/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "remapper_muxer_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/ncjt/rg_modes.h>
#include "common.h"

namespace gr {
  namespace ncjt {
    // ------------------------------------------------------------------------
    // Factory
    // ------------------------------------------------------------------------
    remapper_muxer::sptr
    remapper_muxer::make(int phase, int rgmode, int nstrm, bool reencode, bool debug) {
      return gnuradio::make_block_sptr<remapper_muxer_impl>(
        phase,
        rgmode,
        nstrm,
        reencode,
        debug);
    }

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    remapper_muxer_impl::remapper_muxer_impl(int phase,
                                             int rgmode,
                                             int nstrm,
                                             bool reencode,
                                             bool debug)
      : gr::tagged_stream_block(
          "remapper_muxer",
          gr::io_signature::make(1, 1, sizeof(uint8_t)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          "packet_len"),
        d_phase(phase),
        d_nstrm(nstrm),
        d_modtype(-1),
        d_code_rate(-1),
        d_reencode(reencode),
        d_debug(debug),
        d_seqno(0) {

      if (d_debug) {
        std::cout << "[CONSTRUCTOR remapper_muxer_impl (phase=" << d_phase
            << ", rgmode=" << rgmode
            << ", nstrm=" << d_nstrm
            << ", debug=" << d_debug << ")]"
            << std::endl;
      }

      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else {
        d_n_ofdm_syms = RG_NUM_OFDM_SYM[rgmode];
        d_sd_num = RG_NUM_DATA_SC[rgmode];
      }

      // Constellations
      d_constellation_qpsk = &CONST_QPSK;
      d_constellation_16qam = &CONST_16QAM;
      d_constellation_64qam = &CONST_64QAM;
      d_constellation_256qam = &CONST_256QAM;

      set_tag_propagation_policy(block::TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    remapper_muxer_impl::~remapper_muxer_impl() {
    }

    int remapper_muxer_impl::calculate_output_stream_length(
      const gr_vector_int &ninput_items) {
      int noutput_items = d_n_ofdm_syms * d_sd_num * d_nstrm; // 64 QPSK symbols + 2016 QAM symbols
      if (d_debug) {
        std::cout << "[remapper_muxer_impl::calculate_output_stream_length] "
            << "ninput_items[0]=" << ninput_items[0] << ", noutput_items=" << noutput_items << std::endl;
      }
      return noutput_items;
    }

    int remapper_muxer_impl::work(int noutput_items,
                                  gr_vector_int &ninput_items,
                                  gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items) {
      cc++;
      if (true) {
        std::cout << "[remapper_muxer_impl::work(" << cc
            << ")] Called, noutput_items=" << noutput_items
            << ", ninput_items[0]=" << ninput_items[0] << "" << std::endl;
      }
      gr_complex *out0 = static_cast<gr_complex *>(output_items[0]);

      std::vector<gr::tag_t> tags;
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_seqno"));
      d_seqno = pmt::to_uint64(tags[0].value);

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_modtype_phase1"));
      if (!tags.size())
        throw std::runtime_error(
          "[remapper_muxer_impl] ERROR: The rx_modtype_phase1 tag was not found in input stream");
      int phase1_modtype = pmt::to_uint64(tags[0].value);

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_modtype_phase2"));
      if (!tags.size())
        throw std::runtime_error(
          "[remapper_muxer_impl] ERROR: The rx_modtype_phase2 tag was not found in input stream");
      int phase2_modtype = pmt::to_uint64(tags[0].value);

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_modtype_phase3"));
      if (!tags.size())
        throw std::runtime_error(
          "[remapper_muxer_impl] ERROR: The rx_modtype_phase3 tag was not found in input stream");
      int phase3_modtype = pmt::to_uint64(tags[0].value);
      
      // d_modtype = pmt::to_uint64(tags[0].value);

      int d_modtype = -1;
      if (d_phase == 2) {
        // Phase 2: use phase2_modtype
        d_modtype = phase2_modtype;
      } else if (d_phase == 3) {
        // Phase 3: use phase3_modtype
        d_modtype = phase3_modtype;
      } else {
        throw std::runtime_error(
          "[remapper_muxer_impl] ERROR: Invalid phase " + std::to_string(d_phase));
      }

      // How many data symbols per frame?
      int frame_data_syms = (d_nstrm * d_n_ofdm_syms * d_sd_num) - (d_nstrm * 64);

      ///////
      std::vector<uint8_t> raw_in_bits; // bits to feed into LDPC or direct
      // Unpack bits from each input byte into phase2_modtype bits
      int in_bits_needed = ninput_items[0] * phase2_modtype;
      raw_in_bits.resize(in_bits_needed);
      for (int i = 0; i < ninput_items[0]; i++) {
        uint8_t val = static_cast<const uint8_t *>(input_items[0])[i];
        for (int b = 0; b < phase2_modtype; b++) {
          int shift = (phase2_modtype - 1) - b;
          raw_in_bits[i * phase2_modtype + b] = (val >> shift) & 0x1;
        }
      }
      ///////



      return 0;
      // // The final number of bits to be mapped onto data symbols:
      // int frame_data_bits_out = frame_data_syms * d_modtype;

      // // Fill d_bit_buffer with zeros
      // d_bit_buffer.resize(frame_data_bits_out, 0);

      // // Unpack bits from each input byte into phase2_modtype bits
      // int nbytes_in = frame_data_syms;
      // for (int i = 0; i < nbytes_in; i++) {
      //   uint8_t val = static_cast<const uint8_t *>(input_items[0])[i];
      //   for (int b = 0; b < phase2_modtype; b++) {
      //     int shift = (phase2_modtype - 1) - b;
      //     d_bit_buffer[i * phase2_modtype + b] = (val >> shift) & 0x1;
      //   }
      // }

      // get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_raw_ctrl"));
      // if (!tags.size())
      //   throw std::runtime_error("[remapper_muxer_impl] ERROR: No rx_raw_ctrl tag found in input stream");
      // const unsigned __int128 *raw_ctrl = reinterpret_cast<const unsigned __int128 *>(pmt::blob_data(tags[0].value));

      // CTRL ctrl(d_debug);
      // ctrl.set_raw(*raw_ctrl);

      // // Write SNRs (dB) for each RB into extended field

      // get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("snr_rbs_db"));
      // std::vector<float> snr_values = pmt::f32vector_elements(tags[0].value);
      // size_t num_snr_elements = snr_values.size();

      // uint64_t packed = quantize_snrs(snr_values.data(), num_snr_elements,
      //                                 B0, Bd,
      //                                 MIN_ABS, MAX_ABS,
      //                                 MIN_DIFF, MAX_DIFF);
      // // std::cout << "[remapper_muxer_impl] EXTENDED: " << std::hex << packed << std::dec << std::endl;
      // ctrl.set_extended(packed);

      // ctrl.set_seq_number(d_seqno); // Do not remove!

      // // ctrl.set_data_checksum(data_crc16);
      // std::vector<gr_complex> ctrl_syms = ctrl.pack_and_modulate_16qam();

      // // Place those 64 QPSK symbols, repeated per stream
      // for (int i = 0; i < d_nstrm; i++) {
      //   for (int j = 0; j < 64; j++) {
      //     out0[j * d_nstrm + i] = ctrl_syms[j];
      //   }
      // }

      // // Data portion offset in the output (in symbols)
      // int data_sym_offset = d_nstrm * 64;

      // // The number of "OFDM symbol periods" in the data portion
      // int num_periods = frame_data_syms / d_nstrm;

      // std::vector<uint8_t> used_bits(d_bit_buffer.begin(),
      //                                d_bit_buffer.begin() + frame_data_bits_out);

      // assert(used_bits.size() == (size_t)frame_data_bits_out);

      // // Map used_bits -> QAM symbols
      // for (int k = 0; k < num_periods; k++) {
      //   int period_offset = k * d_nstrm * d_modtype;
      //   for (int s = 0; s < d_nstrm; s++) {
      //     int val = 0;
      //     // gather bits for this symbol
      //     for (int j = 0; j < d_modtype; j++) {
      //       int bit_index = period_offset + s + j * d_nstrm;
      //       val |= (used_bits[bit_index] << j);
      //     }
      //     // map val to a constellation point
      //     gr_complex sym;
      //     switch (d_modtype) {
      //       case 2:
      //         sym = (*d_constellation_qpsk)[val];
      //         break;
      //       case 4:
      //         sym = (*d_constellation_16qam)[val];
      //         break;
      //       case 6:
      //         sym = (*d_constellation_64qam)[val];
      //         break;
      //       case 8:
      //         sym = (*d_constellation_256qam)[val];
      //         break;
      //       default:
      //         throw std::runtime_error("Invalid modulation type");
      //     }
      //     // place symbol into output in interleaved fashion
      //     out0[data_sym_offset + k * d_nstrm + s] = sym;
      //   }
      // }

      // int total_syms_out = d_nstrm * d_n_ofdm_syms * d_sd_num;
      // add_item_tag(0, nitems_written(0), pmt::string_to_symbol("packet_len"),
      //              pmt::from_long(total_syms_out));
      // add_item_tag(0, nitems_written(0), pmt::string_to_symbol("seqno"),
      //              pmt::from_long(d_seqno));

      // d_bit_buffer.clear();

      // // Bump sequence number (to be overridden by tags if valid control packet)
      // d_seqno++;

      // return total_syms_out;
    }
  } /* namespace ncjt */
} /* namespace gr */
