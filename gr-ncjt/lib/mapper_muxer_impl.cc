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

namespace gr
{
  namespace ncjt
  {

    ////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////
    mapper_muxer::sptr mapper_muxer::make(int nstrm,
                                          int modtype,
                                          int n_ofdm_syms,
                                          int sd_num,
                                          bool use_polar,
                                          int code_rate,
                                          bool deterministic_input,
                                          bool debug)
    {
      return gnuradio::make_block_sptr<mapper_muxer_impl>(
          nstrm, modtype, n_ofdm_syms, sd_num, use_polar,
          code_rate, deterministic_input, debug);
    }

    ////////////////////////////////////////////////////////////////////
    // mapper_muxer_impl ctor
    ////////////////////////////////////////////////////////////////////
    mapper_muxer_impl::mapper_muxer_impl(int nstrm,
                                         int modtype,
                                         int n_ofdm_syms,
                                         int sd_num,
                                         bool use_polar,
                                         int code_rate,
                                         bool deterministic_input,
                                         bool debug)
        : gr::block("mapper_muxer",
                    gr::io_signature::make((int)(!deterministic_input), (int)(!deterministic_input), sizeof(uint8_t)),
                    gr::io_signature::make(1, 1, sizeof(gr_complex))),
          d_nstrm(nstrm),
          d_modtype(modtype),
          d_n_ofdm_syms(n_ofdm_syms),
          d_sd_num(sd_num),
          d_use_polar(use_polar),
          d_code_rate(code_rate),
          d_deterministic_input(deterministic_input),
          d_debug(debug),
          d_seqno(0)
    {
      cc = 0;
      if (d_debug)
      {
        std::cout << "[CONSTRUCTOR mapper_muxer_impl"
                  << ", d_nstrm=" << d_nstrm << ", d_modtype=" << d_modtype
                  << ", d_n_ofdm_syms=" << d_n_ofdm_syms
                  << ", d_sd_num=" << d_sd_num
                  << ", d_use_polar=" << d_use_polar
                  << ", d_code_rate=" << d_code_rate
                  << ", d_deterministic_input=" << d_deterministic_input
                  << ", d_debug=" << d_debug << ")]" << std::endl;
      }

      if (d_nstrm < 1 || d_nstrm > 4)
        throw std::runtime_error("mapper_muxer: invalid nstrm (1..4).");
      if (d_modtype != 2 && d_modtype != 4 && d_modtype != 6 && d_modtype != 8)
        throw std::runtime_error("mapper_muxer: invalid modtype (2,4,6,8).");
      if (n_ofdm_syms <= 0)
        throw std::runtime_error("mapper_muxer: n_ofdm_syms must be > 0.");
      if (sd_num <= 0)
        throw std::runtime_error("mapper_muxer: sd_num must be > 0.");
      if (d_code_rate < 0 || d_code_rate > 7)
        throw std::runtime_error("mapper_muxer: code_rate must be in [0,7].");
      if (d_debug)
      {
        if (code_rate == 0)
          std::cout << "[mapper_muxer_impl] Code rate: Disabled (no LDPC)."
                    << std::endl;
        else
          std::cout << "[mapper_muxer_impl] Code rate index: " << code_rate
                    << " => " << code_rates[d_code_rate] << std::endl;
      }

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
      if (encoder_factory && matcher_factory)
      {
        d_ldpc_encoder = encoder_factory->create();
        d_ldpc_matcher = matcher_factory->create();
      }
      else
      {
        throw std::runtime_error(
            "[mapper_muxer_impl] ERROR: LDPC factories could not be created.");
      }

      message_port_register_in(pmt::intern("sel_mod_type"));
      set_msg_handler(pmt::intern("sel_mod_type"), [this](pmt::pmt_t msg)
                      {
    if (pmt::is_number(msg)) {
      int new_modtype = pmt::to_long(msg);
      if (new_modtype == 2 || new_modtype == 4 || new_modtype == 6 ||
          new_modtype == 8) {
        d_modtype = new_modtype;
        std::cout << "[mapper_muxer_impl] Updated d_modtype to " << d_modtype
                  << std::endl;
      }
    } });

      message_port_register_in(pmt::intern("sel_code_rate"));
      set_msg_handler(pmt::intern("sel_code_rate"), [this](pmt::pmt_t msg)
                      {
    if (pmt::is_number(msg)) {
      int new_cr = pmt::to_long(msg);
      if (new_cr >= 0 && new_cr <= 7) {
        d_code_rate = new_cr;
        std::cout << "[mapper_muxer_impl] Updated d_code_rate to "
                  << d_code_rate << std::endl;
      }
    } });
    }

    ////////////////////////////////////////////////////////////////////
    // forecast()
    ////////////////////////////////////////////////////////////////////
    void mapper_muxer_impl::forecast(int noutput_items,
                                     gr_vector_int &ninput_items_required)
    {
      int needed_items = 0;
      if (!d_deterministic_input)
      {
        // Always subtract space for control symbols (64 QPSK per stream).
        needed_items = d_nstrm * d_n_ofdm_syms * d_sd_num * d_modtype
                       - d_nstrm * 64 * d_modtype;
        if (d_code_rate > 0)
        {
          double R = code_rates[d_code_rate];
          needed_items = int(std::floor(needed_items * R));
          needed_items = needed_items + (d_modtype - (needed_items % d_modtype));
        }
        // subtract any bits already buffered:
        needed_items -= d_bit_buffer.size();
        if (needed_items < 1)
        {
          needed_items = 0;
        }
      }
      if (d_debug)
      {
        std::cout << " *[mapper_muxer_impl::forecast(" << cc << ")] Forecasting "
                  << needed_items << " input items required for " << noutput_items
                  << " output items (deterministic=" << d_deterministic_input << ")" << std::endl;
      }
      ninput_items_required[0] = needed_items;
    }

    ////////////////////////////////////////////////////////////////////
    // general_work()
    ////////////////////////////////////////////////////////////////////
    int mapper_muxer_impl::general_work(int noutput_items,
                                        gr_vector_int &ninput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
    {
      cc++;
      if (d_debug)
      {
        std::cout << "[mapper_muxer_impl::general_work(" << cc
                  << ")] Called, noutput_items=" << noutput_items
                  << ", ninput_items[0]=" << ninput_items[0] << std::endl;
      }

      // Output port 0: complex symbols
      gr_complex *out0 = static_cast<gr_complex *>(output_items[0]);

      //--------------------------------------------------------------------
      //  IF NOT DETERMINISTIC:
      //    - read bits from the input stream, push them into d_bit_buffer
      //  IF DETERMINISTIC:
      //    - skip reading from input; we will generate bits ourselves
      //--------------------------------------------------------------------

      if (!d_deterministic_input)
      {
        const uint8_t *in_bits = static_cast<const uint8_t *>(input_items[0]);
        // Pull in all available bits from input
        int inp_bits_avail = ninput_items[0];
        d_bit_buffer.insert(d_bit_buffer.end(), in_bits, in_bits + inp_bits_avail);

        // We have consumed all input bits we've taken
        consume_each(inp_bits_avail);

        if (d_debug)
        {
          std::cout << " [mapper_muxer_impl::general_work(" << cc
                    << ")] Pulled " << inp_bits_avail
                    << " bits from input; d_bit_buffer.size()="
                    << d_bit_buffer.size() << std::endl;
        }
      }
      // else (d_deterministic_input==true): do nothing here

      //--------------------------------------------------------------------
      // Calculate how many *coded bits* we need to fill one full frame
      //--------------------------------------------------------------------
      int frame_data_syms = (d_nstrm * d_n_ofdm_syms * d_sd_num) - (d_nstrm * 64);

      // The final number of bits to be mapped onto data symbols:
      int frame_data_bits_out = frame_data_syms * d_modtype;

      // If code rate is nonzero, we only need
      // in_bits_needed = floor(frame_data_bits_out * R)
      // where R is the "information ratio".
      int in_bits_needed = frame_data_bits_out; // default if no coding
      if (d_code_rate > 0)
      {
        double R = code_rates[d_code_rate];
        in_bits_needed = int(std::floor(frame_data_bits_out * R));
        in_bits_needed = in_bits_needed + (d_modtype - (in_bits_needed % d_modtype));
        if (d_debug)
        {
          std::cout << "in_bits_needed=" << in_bits_needed
                    << "in_bits_needed%modtype=" << in_bits_needed % d_modtype << std::endl;
        }
      }

      //--------------------------------------------------------------------
      // If deterministic_input==true, generate exactly 'in_bits_needed' bits
      // for this frame using a PRNG seeded with d_seqno.
      //--------------------------------------------------------------------
      std::vector<uint8_t> raw_in_bits; // bits to feed into LDPC or direct
      if (d_deterministic_input)
      {
        raw_in_bits.resize(in_bits_needed);
        // ADDED FOR DETERMINISTIC INPUT
        // Use seed = d_seqno to get a reproducible random stream for each frame
        std::mt19937 gen(d_seqno);
        std::uniform_int_distribution<int> dist(0, 1);
        for (int i = 0; i < in_bits_needed; i++)
        {
          raw_in_bits[i] = dist(gen) & 0x1;
        }
        // END DETERMINISTIC
      }
      else
      {
        // If we are NOT deterministic, we need to check if d_bit_buffer
        // has enough bits. If not, return 0 (we cannot produce a new packet).
        if ((int)d_bit_buffer.size() < in_bits_needed)
        {
          if (d_debug)
          {
            std::cout << " [mapper_muxer_impl::general_work(" << cc
                      << ")] Not enough bits; have " << d_bit_buffer.size()
                      << ", need " << in_bits_needed << std::endl;
          }
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
      int data_modtype_for_ctrl = 0;
      switch (d_modtype)
      {
      case 2:
        data_modtype_for_ctrl = 0; // QPSK
        break;
      case 4:
        data_modtype_for_ctrl = 1; // 16QAM
        break;
      case 6:
        data_modtype_for_ctrl = 2; // 64QAM
        break;
      case 8:
        data_modtype_for_ctrl = 3; // 256QAM
        break;
      default:
        throw std::runtime_error("Invalid modulation type");
      }
      CTRL ctrl(d_debug);
      ctrl.set_seq_number(d_seqno);
      ctrl.set_nstrm_phase2(d_nstrm);
      ctrl.set_mod_type_phase2(data_modtype_for_ctrl);
      ctrl.set_coding_rate_phase2(d_code_rate);
      // TODO: We might want to have different MCS for phase 3
      ctrl.set_nstrm_phase3(d_nstrm);
      ctrl.set_mod_type_phase3(data_modtype_for_ctrl);
      ctrl.set_coding_rate_phase3(d_code_rate);
      ctrl.set_data_checksum(data_crc16);

      std::vector<gr_complex> ctrl_syms = ctrl.pack_and_modulate_qpsk();

      // Place those 64 QPSK symbols, repeated per stream
      for (int i = 0; i < d_nstrm; i++)
      {
        for (int j = 0; j < 64; j++)
        {
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
      if (d_code_rate == 0)
      {
        // No coding, just copy the needed bits
        used_bits.insert(used_bits.end(), raw_in_bits.begin(),
                         raw_in_bits.begin() + frame_data_bits_out);
      }
      else
      {

        ////// New segmented processing
        int seg_out_size = d_n_ofdm_syms * d_sd_num - 64;
        int num_segs = frame_data_bits_out / seg_out_size;
        int base_in_bits_per_seg = in_bits_needed / num_segs;
        int remainder = in_bits_needed % num_segs;
        if (d_debug)
        {
          std::cout << " [mapper_muxer_impl::general_work(" << cc
                    << ")] Segmented processing: \n\t\tin_bits_needed="
                    << in_bits_needed << "\n\t\tseg_out_size=" << seg_out_size
                    << "\n\t\tnum_segs=" << num_segs
                    << "\n\t\tbase_in_bits_per_seg=" << base_in_bits_per_seg
                    << "\n\t\tremainder=" << remainder << std::endl;
        }

        used_bits.clear();
        used_bits.reserve(frame_data_bits_out);

        int start_idx = 0;
        for (int seg = 0; seg < num_segs; seg++)
        {
          int in_bits_per_seg =
              base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
          std::vector<uint8_t> raw_in(raw_in_bits.begin() + start_idx,
                                      raw_in_bits.begin() + start_idx + in_bits_per_seg);
          if (d_debug)
          {
            std::cout << "\t\t >> Segment " << seg << " (start_idx=" << start_idx
                      << ", in_bits_per_seg=" << in_bits_per_seg << ")"
                      << std::endl;
          }

          auto seg_encoded = gr::ncjt::ldpc_encode(raw_in, seg_out_size,
                                                   d_ldpc_encoder.get(),
                                                   d_ldpc_matcher.get());
          used_bits.insert(used_bits.end(), seg_encoded.begin(), seg_encoded.end());
          start_idx += in_bits_per_seg;
        }

        assert(used_bits.size() == (size_t)frame_data_bits_out);
      }

      assert(used_bits.size() == (size_t)frame_data_bits_out);

      // Map used_bits -> QAM symbols
      for (int k = 0; k < num_periods; k++)
      {
        int period_offset = k * d_nstrm * d_modtype;
        for (int s = 0; s < d_nstrm; s++)
        {
          int val = 0;
          // gather bits for this symbol
          for (int j = 0; j < d_modtype; j++)
          {
            int bit_index = period_offset + s + j * d_nstrm;
            val |= (used_bits[bit_index] << j);
          }
          // map val to a constellation point
          gr_complex sym;
          switch (d_modtype)
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

      if (d_debug)
      {
        std::cout << " [mapper_muxer_impl::general_work(" << cc << ")] code_rate="
                  << code_rates[d_code_rate]
                  << ", in_bits_needed=" << in_bits_needed
                  << " => " << frame_data_bits_out
                  << " coded bits, producing " << total_syms_out
                  << " QAM symbols." << std::endl;
      }

      // If we used real input bits (non-deterministic), remove them from buffer
      if (!d_deterministic_input)
      {
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
