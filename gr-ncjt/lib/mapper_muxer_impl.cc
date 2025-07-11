/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mapper_muxer_impl.h"
#include <gnuradio/io_signature.h>
#include <iostream>  // debug prints
#include <pmt/pmt.h> // PMT tags
#include <stdexcept>
#include <random>
#include "rg_modes.h"

namespace gr {
  namespace ncjt {
    ////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////
    mapper_muxer::sptr mapper_muxer::make(int rgmode,
                                          int nstrm,
                                          int phase1_modtype,
                                          int phase2_modtype,
                                          int phase3_modtype,
                                          int code_rate,
                                          bool deterministic_input,
                                          bool debug) {
      return gnuradio::make_block_sptr<mapper_muxer_impl>(
        rgmode, nstrm, phase1_modtype, phase2_modtype, phase3_modtype,
        code_rate, deterministic_input, debug);
    }

    ////////////////////////////////////////////////////////////////////
    // mapper_muxer_impl ctor
    ////////////////////////////////////////////////////////////////////
    mapper_muxer_impl::mapper_muxer_impl(int rgmode,
                                         int nstrm,
                                         int phase1_modtype,
                                         int phase2_modtype,
                                         int phase3_modtype,
                                         int code_rate,
                                         bool deterministic_input,
                                         bool debug)
      : gr::block("mapper_muxer",
                  gr::io_signature::make((int) (!deterministic_input), (int) (!deterministic_input),
                                         sizeof(uint8_t)),
                  gr::io_signature::make(1, 1, sizeof(gr_complex))),
        d_nstrm(nstrm),
        d_phase1_modtype(phase1_modtype),
        d_phase2_modtype(phase2_modtype),
        d_phase3_modtype(phase3_modtype),
        d_code_rate(code_rate),
        d_deterministic_input(deterministic_input),
        d_debug(debug),
        d_seqno(0) {
      cc = 0;
      NCJT_LOG(d_debug, "rg_mode=" << rgmode
            << ", d_nstrm=" << d_nstrm
            << ", d_phase1_modtype=" << d_phase1_modtype
            << ", d_phase2_modtype=" << d_phase2_modtype
            << ", d_phase3_modtype=" << d_phase3_modtype
            << ", d_code_rate=" << d_code_rate
            << ", d_deterministic_input=" << d_deterministic_input);

      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else {
        d_n_ofdm_syms = RG_NUM_OFDM_SYM[rgmode];
        d_sd_num = RG_NUM_DATA_SC[rgmode];
      }
      if (d_nstrm < 1 || d_nstrm > 4)
        throw std::runtime_error("mapper_muxer: invalid nstrm (1..4).");
      if (!d_valid_mod_types.count(d_phase1_modtype))
        throw std::runtime_error("mapper_muxer: invalid modtype (2,4,6,8).");
      if (!d_valid_mod_types.count(d_phase2_modtype))
        throw std::runtime_error("mapper_muxer: invalid modtype (2,4,6,8).");
      if (!d_valid_mod_types.count(d_phase3_modtype))
        throw std::runtime_error("mapper_muxer: invalid modtype (2,4,6,8).");
      // if (d_phase2_modtype < d_phase1_modtype)
      //   throw std::runtime_error("mapper_muxer: phase2_modtype must be >= phase1_modtype.");
      // if (d_phase3_modtype < d_phase2_modtype)
      //   throw std::runtime_error("mapper_muxer: phase3_modtype must be >= phase2_modtype.");
      if (d_phase1_modtype < d_phase2_modtype)
        throw std::runtime_error("mapper_muxer: phase1_modtype must be >= phase2_modtype.");
      if (d_phase3_modtype < d_phase2_modtype)
        throw std::runtime_error("mapper_muxer: phase3_modtype must be >= phase2_modtype.");
      if (d_code_rate < 0 || d_code_rate > 7)
        throw std::runtime_error("mapper_muxer: code_rate must be in [0,7].");

      if (code_rate == 0)
        NCJT_LOG(d_debug, "Code rate: Disabled (no LDPC).");
      else
        NCJT_LOG(d_debug, "Code rate index: " << code_rate
              << " => " << code_rates[d_code_rate]);

      // Constellations
      d_constellation_qpsk = &CONST_QPSK;
      d_constellation_16qam = &CONST_16QAM;
      d_constellation_64qam = &CONST_64QAM;
      d_constellation_256qam = &CONST_256QAM;

      // We'll produce the entire frame of symbols in one go on port 0
      set_output_multiple(d_n_ofdm_syms * d_sd_num * d_nstrm);

      // We can set the minimum output buffer for the first port:
      set_min_output_buffer(0, d_n_ofdm_syms * d_sd_num * d_nstrm);

      // We are manually controlling tags, so do not propagate input tags
      // automatically.
      set_tag_propagation_policy(block::TPP_DONT);

      auto encoder_factory = srsran::create_ldpc_encoder_factory_sw("avx2");
      auto matcher_factory = srsran::create_ldpc_rate_matcher_factory_sw();
      if (encoder_factory && matcher_factory) {
        d_ldpc_encoder = encoder_factory->create();
        d_ldpc_matcher = matcher_factory->create();
      } else {
        throw std::runtime_error(
          "[mapper_muxer_impl] ERROR: LDPC factories could not be created.");
      }

      message_port_register_in(pmt::intern("s_modtypes"));
      set_msg_handler(pmt::intern("s_modtypes"),
                      [this](pmt::pmt_t msg) {
                        if (pmt::is_number(msg)) {
                          int new_modtypes = pmt::to_long(msg);

                          int d_new_phase1_modtype = (new_modtypes >> 8) & 0xF;
                          int d_new_phase2_modtype = (new_modtypes >> 4) & 0xF;
                          int d_new_phase3_modtype = new_modtypes & 0xF;

                          // If valid values are passed
                          if (d_valid_mod_types.count(d_new_phase1_modtype) &&
                              d_valid_mod_types.count(d_new_phase2_modtype) &&
                              d_valid_mod_types.count(d_new_phase3_modtype)) {
                            if ((d_new_phase2_modtype <= d_new_phase1_modtype) &&
                                (d_new_phase2_modtype <= d_new_phase3_modtype)) {
                              d_phase1_modtype = d_new_phase1_modtype;
                              d_phase2_modtype = d_new_phase2_modtype;
                              d_phase3_modtype = d_new_phase3_modtype;
                              std::cout << "[mapper_muxer_impl] Updated d_phase1_modtype to "
                                  << d_phase1_modtype << ", d_phase2_modtype to "
                                  << d_phase2_modtype << ", d_phase3_modtype to "
                                  << d_phase3_modtype << std::endl;
                            } else {
                              throw std::runtime_error(
                                "mapper_muxer: Phase 2 modtype must be less than or equal to both phase 1 and phase 3 modtypes. "
                                "(phase1:" + std::to_string(d_new_phase1_modtype) +
                                ", phase2:" + std::to_string(d_new_phase2_modtype) +
                                ", phase3:" + std::to_string(d_new_phase3_modtype) + ")."
                                );
                            }
                          } else {
                            throw std::runtime_error("mapper_muxer: Invalid received modtype message.");
                          }
                        } else {
                          throw std::runtime_error("mapper_muxer: Invalid received modtype message.");
                        }
                      });

      message_port_register_in(pmt::intern("s_code_rate"));
      set_msg_handler(pmt::intern("s_code_rate"), [this](pmt::pmt_t msg) {
        if (pmt::is_number(msg)) {
          int new_cr = pmt::to_long(msg);
          if (new_cr >= 0 && new_cr <= 7) {
            d_code_rate = new_cr;
            std::cout << "[mapper_muxer_impl] Updated d_code_rate to "
                << d_code_rate << std::endl;
          }
        }
      });
    }

    ////////////////////////////////////////////////////////////////////
    // forecast()
    ////////////////////////////////////////////////////////////////////
    void mapper_muxer_impl::forecast(int noutput_items,
                                     gr_vector_int &ninput_items_required) {
      int needed_items = 0;
      if (!d_deterministic_input) {
        // Always subtract space for control symbols (64 QPSK per stream).
        // Deprecated: needed_items = d_nstrm * d_n_ofdm_syms * d_sd_num * d_phase1_modtype - d_nstrm * 64 * d_phase1_modtype;
        // TODO: Consider different RG dimensions across phases.
        needed_items = (d_n_ofdm_syms * d_sd_num - 64) * d_nstrm * d_phase2_modtype;
        if (d_code_rate > 0) {
          double R = code_rates[d_code_rate];
          needed_items = int(std::floor(needed_items * R));
          needed_items = needed_items + (d_phase1_modtype - (needed_items % d_phase1_modtype)); // TODO: Not sure if this is needed
        }
        // subtract any bits already buffered:
        needed_items -= d_bit_buffer.size();
        if (needed_items < 1) {
          needed_items = 0;
        }
      }
      NCJT_LOG(d_debug, "Forecasting " << needed_items
            << " input items required for " << noutput_items
            << " output items (deterministic=" << d_deterministic_input << ")");
      ninput_items_required[0] = needed_items;
    }

    ////////////////////////////////////////////////////////////////////
    // general_work()
    ////////////////////////////////////////////////////////////////////
    int mapper_muxer_impl::general_work(int noutput_items,
                                        gr_vector_int &ninput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items) {
      cc++;

      NCJT_LOG(d_debug, "Called, noutput_items=" << noutput_items
                        << ", ninput_items[0]=" << ninput_items[0]);

      // Output port 0: complex symbols
      gr_complex *out0 = static_cast<gr_complex *>(output_items[0]);

      //--------------------------------------------------------------------
      //  IF NOT DETERMINISTIC:
      //    - read bits from the input stream, push them into d_bit_buffer
      //  IF DETERMINISTIC:
      //    - skip reading from input; we will generate bits ourselves
      //--------------------------------------------------------------------

      if (!d_deterministic_input) {
        const uint8_t *in_bits = static_cast<const uint8_t *>(input_items[0]);
        // Pull in all available bits from input
        int inp_bits_avail = ninput_items[0];
        d_bit_buffer.insert(d_bit_buffer.end(), in_bits, in_bits + inp_bits_avail);

        // We have consumed all input bits we've taken
        consume_each(inp_bits_avail);

        NCJT_LOG(d_debug, "Pulled " << inp_bits_avail << " bits from input; d_bit_buffer.size()=" << d_bit_buffer.size());
      }
      // else (d_deterministic_input==true): do nothing here

      //--------------------------------------------------------------------
      // Calculate how many *coded bits* we need to fill one full frame
      //--------------------------------------------------------------------
      int frame_data_syms = d_nstrm * (d_n_ofdm_syms * d_sd_num - 64); // TODO: What if RG dimensions differ across phases?

      // The final number of bits to be mapped onto data symbols:
      // Deprecated: int frame_data_bits_out = frame_data_syms * d_phase1_modtype;
      int frame_data_bits_out_capacity = frame_data_syms * d_phase1_modtype;
      int frame_data_bits_phase2_out = frame_data_syms * d_phase2_modtype;

      // If code rate is nonzero, we need to calculate the number of info bits.
      int in_bits_needed = frame_data_bits_phase2_out; // default if no coding
      if (d_code_rate > 0) {
        double R = code_rates[d_code_rate];
        in_bits_needed = int(std::floor(frame_data_bits_phase2_out * R));
        in_bits_needed = in_bits_needed + (d_phase2_modtype - (in_bits_needed % d_phase2_modtype));

        NCJT_LOG(d_debug, "in_bits_needed=" << in_bits_needed
                  << "in_bits_needed%modtype=" << in_bits_needed % d_phase2_modtype);
      }

      //--------------------------------------------------------------------
      // If deterministic_input==true, generate exactly 'in_bits_needed' bits
      // for this frame using a PRNG seeded with d_seqno.
      //--------------------------------------------------------------------
      std::vector<uint8_t> raw_in_bits; // bits to feed into LDPC or direct
      if (d_deterministic_input) {
        raw_in_bits.resize(in_bits_needed);
        // ADDED FOR DETERMINISTIC INPUT
        // Use seed = d_seqno to get a reproducible random stream for each frame
        std::mt19937 gen(d_seqno);
        std::uniform_int_distribution<int> dist(0, 1);
        for (int i = 0; i < in_bits_needed; i++) {
          raw_in_bits[i] = dist(gen) & 0x1;
        }
        std::cout << "@@@" 
                  << "raw_in_bits.size(): " << raw_in_bits.size() << std::endl;
        // END DETERMINISTIC
      } else {
        // If we are NOT in deterministic mode, we need to check if d_bit_buffer
        // has enough bits. If not, return 0 (we cannot produce a new packet).
        if ((int) d_bit_buffer.size() < in_bits_needed) {
          NCJT_LOG(d_debug, "Not enough bits; have " << d_bit_buffer.size()
                << ", need " << in_bits_needed);
          return 0; // no output
        }
        // copy from d_bit_buffer into raw_in_bits
        raw_in_bits.insert(raw_in_bits.end(),
                           d_bit_buffer.begin(),
                           d_bit_buffer.begin() + in_bits_needed);
      }

      //--------------------------------------------------------------------
      // Build control portion
      //--------------------------------------------------------------------
      uint16_t data_crc16 = gr::ncjt::compute_crc16(&raw_in_bits[0], in_bits_needed);

      // Build the control portion
      int data_modtype_ind_phase1 = modtype_bits_to_index(d_phase1_modtype);
      int data_modtype_ind_phase2 = modtype_bits_to_index(d_phase2_modtype);
      int data_modtype_ind_phase3 = modtype_bits_to_index(d_phase3_modtype);

      CTRL ctrl(d_debug);
      ctrl.set_seq_number(d_seqno);
      //
      ctrl.set_nstrm_phase1(d_nstrm);
      ctrl.set_mod_type_phase1(data_modtype_ind_phase1);
      ctrl.set_coding_rate_phase1(d_code_rate);
      //
      ctrl.set_nstrm_phase2(d_nstrm);
      ctrl.set_mod_type_phase2(data_modtype_ind_phase2);
      ctrl.set_coding_rate_phase2(d_code_rate);
      //
      ctrl.set_nstrm_phase3(d_nstrm);
      ctrl.set_mod_type_phase3(data_modtype_ind_phase3);
      ctrl.set_coding_rate_phase3(d_code_rate);
      //
      ctrl.set_data_checksum(data_crc16);

      std::vector<gr_complex> ctrl_syms = ctrl.pack_and_modulate_qpsk();

      // Place those 64 QPSK symbols, repeated per stream
      for (int i = 0; i < d_nstrm; i++) {
        for (int j = 0; j < 64; j++) {
          out0[j * d_nstrm + i] = ctrl_syms[j];
        }
      }

      // Data portion offset in the output (in symbols)
      int data_sym_offset = d_nstrm * 64;

      int num_periods = frame_data_syms / d_nstrm; // each period is one symbol from each stream

      //--------------------------------------------------------------------
      // LDPC encode (or pass through) the raw_in_bits so we get 'frame_data_bits_out' bits
      //--------------------------------------------------------------------

      std::vector<uint8_t> used_bits;

      used_bits.clear();
      used_bits.reserve(frame_data_bits_out_capacity);

      if (d_code_rate == 0) {
        // No coding, just copy the needed bits
        used_bits.insert(used_bits.end(), raw_in_bits.begin(),
                         raw_in_bits.begin() + frame_data_bits_phase2_out);
      } else {
        ////// New segmented processing
        int seg_out_size = d_n_ofdm_syms * d_sd_num - 64;
        int num_segs = frame_data_bits_phase2_out / seg_out_size;
        int base_in_bits_per_seg = in_bits_needed / num_segs;
        int remainder = in_bits_needed % num_segs;
        NCJT_LOG(d_debug,
            " (" << cc << ") Segmented processing: "
            << "in_bits_needed=" << in_bits_needed
            << ", seg_out_size=" << seg_out_size
            << ", num_segs=" << num_segs
            << ", base_in_bits_per_seg=" << base_in_bits_per_seg
            << ", remainder=" << remainder
        );

        int start_idx = 0;
        for (int seg = 0; seg < num_segs; seg++) {
          int in_bits_per_seg =
              base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
          std::vector<uint8_t> raw_in(raw_in_bits.begin() + start_idx,
                                      raw_in_bits.begin() + start_idx + in_bits_per_seg);

          NCJT_LOG(d_debug, "\t\t >> Segment " << seg << " (start_idx=" << start_idx
                        << ", in_bits_per_seg=" << in_bits_per_seg << ")");

          auto seg_encoded = gr::ncjt::ldpc_encode(raw_in, seg_out_size,
                                                   d_ldpc_encoder.get(),
                                                   d_ldpc_matcher.get());
          used_bits.insert(used_bits.end(), seg_encoded.begin(), seg_encoded.end());
          start_idx += in_bits_per_seg;
        }
      }

      // Padding: Fill the remaining frame_data_bits_out_capacity with random bits
      std::mt19937 gen_padding(d_seqno+11); // different seed for padding
      std::uniform_int_distribution<int> dist_padding(0, 1);
      while ((int)used_bits.size() < frame_data_bits_out_capacity) {
        used_bits.push_back(dist_padding(gen_padding) & 0x1);
      }

      assert(used_bits.size() == (size_t)frame_data_bits_out_capacity);

      // Map used_bits -> QAM symbols
      for (int k = 0; k < num_periods; k++) {
        int period_offset = k * d_nstrm * d_phase1_modtype;
        for (int s = 0; s < d_nstrm; s++) {
          int val = 0;
          // gather bits for this symbol
          for (int j = 0; j < d_phase1_modtype; j++) {
            int bit_index = period_offset + s + j * d_nstrm;
            val |= (used_bits[bit_index] << j);
          }
          // map val to a constellation point
          gr_complex sym;
          switch (d_phase1_modtype) {
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
          // place symbol into output in interleaved fashion
          out0[data_sym_offset + k * d_nstrm + s] = sym;
        }
      }

      // Add packet_len tag on port #0
      int total_syms_out = d_nstrm * d_n_ofdm_syms * d_sd_num;
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("packet_len"),
                   pmt::from_long(total_syms_out), pmt::string_to_symbol(this->name()));
      // Add seqno tag on port #0
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("seqno"),
                   pmt::from_long(d_seqno), pmt::string_to_symbol(this->name()));

      NCJT_LOG(d_debug, "code_rate="
            << code_rates[d_code_rate]
            << ", in_bits_needed=" << in_bits_needed
            << " => " << frame_data_bits_phase2_out
            << " coded bits, producing " << frame_data_bits_out_capacity
            << " coded bits + padding, producing " << total_syms_out
            << " QAM symbols.");

      // If we used real input bits (non-deterministic), remove them from buffer
      if (!d_deterministic_input) {
        d_bit_buffer.erase(d_bit_buffer.begin(),
                           d_bit_buffer.begin() + in_bits_needed);
      }

      // Bump sequence number
      d_seqno++;

      // produce() call
      produce(0, total_syms_out);

      // We used produce(), so we return WORK_CALLED_PRODUCE
      return WORK_CALLED_PRODUCE;
    }
  } // namespace ncjt
} // namespace gr
