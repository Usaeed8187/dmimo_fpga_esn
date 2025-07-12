/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "demapper_impl.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <gnuradio/expj.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <iostream>
#include <pmt/pmt.h>
#include <stdexcept>
#include "rg_modes.h"
#include "common.h"
#include <random>

namespace gr
{
  namespace ncjt
  {
    //////////////////////////////////////////////////////////////////////
    // Factory
    //////////////////////////////////////////////////////////////////////
    demapper::sptr demapper::make(int rgmode,
                                  bool coded,
                                  bool deterministic_input,
                                  bool debug)
    {
      return gnuradio::make_block_sptr<demapper_impl>(rgmode, coded, deterministic_input, debug);
    }

    //////////////////////////////////////////////////////////////////////
    // Constructor
    //////////////////////////////////////////////////////////////////////
    demapper_impl::demapper_impl(int rgmode,
                                 bool coded,
                                 bool deterministic_input,
                                 bool debug)
        : gr::block("demapper",
                    gr::io_signature::make(2, 2, sizeof(gr_complex)),
                    gr::io_signature::make(1, 2, sizeof(uint8_t))),
          d_output_raw(coded),
          d_deterministic_input(deterministic_input),
          d_debug(debug),
          cc(0),
          d_rgmode(rgmode),
          d_current_phase(0),
          d_last_nstrm(1),
          d_last_modtype(2),
          d_last_coding_rate(0),
          d_last_pkt_len(0),
          d_last_seq_no(0),
          d_last_syms_per_stream(0),
          d_last_ctrl_ok(true),
          d_found_rx_checksum(false),
          d_rx_data_checksum(0),
          d_coded_buf(nullptr),
          d_coded_len(0),
          d_info_buf(nullptr),
          d_info_len(0)
    {
      NCJT_LOG(d_debug, "d_output_raw=" << d_output_raw
                << ", d_deterministic_input=" << d_deterministic_input);

      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else {
        set_output_multiple(RG_NUM_OFDM_SYM[rgmode] * RG_NUM_DATA_SC[rgmode] - 64);
      }

      set_tag_propagation_policy(TPP_DONT);

      auto decoder_factory = srsran::create_ldpc_decoder_factory_sw("avx2");
      auto dematcher_factory =
          srsran::create_ldpc_rate_dematcher_factory_sw("avx2");

      if (decoder_factory && dematcher_factory)
      {
        d_ldpc_decoder = decoder_factory->create();
        d_ldpc_dematcher = dematcher_factory->create();
      }
      else
      {
        throw std::runtime_error(
            "[demapper_impl] ERROR: LDPC factories could not be created.");
      }

      if (d_deterministic_input)
      {
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
      }
    }

    //////////////////////////////////////////////////////////////////////
    // Destructor
    //////////////////////////////////////////////////////////////////////
    demapper_impl::~demapper_impl()
    {
      if (d_coded_buf)
        free(d_coded_buf);
      if (d_info_buf)
        free(d_info_buf);
    }

    //////////////////////////////////////////////////////////////////////
    // forecast
    //////////////////////////////////////////////////////////////////////
    void demapper_impl::forecast(int noutput_items,
                                 gr_vector_int &ninput_items_required)
    {
      // We do not know modtype in advance, so assume worst-case 8 bits/symbol.
      int needed = noutput_items;

      ninput_items_required[0] = needed;
      ninput_items_required[1] = needed;
    }

    //////////////////////////////////////////////////////////////////////
    // general_work
    //////////////////////////////////////////////////////////////////////
    int demapper_impl::general_work(int noutput_items,
                                    gr_vector_int &ninput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items)
    {
      cc++;

      NCJT_LOG(d_debug, " (" << cc
                << ")] noutput_items=" << noutput_items
                << ", ninput_items[0]=" << ninput_items[0]
                << ", ninput_items[1]=" << ninput_items[1]
                << ", d_last_modtype=" << d_last_modtype
                << ", d_last_coding_rate=" << d_last_coding_rate);

      if (ninput_items[0] != ninput_items[1])
      {
        throw std::runtime_error(
            "[demapper_impl] ERROR: input ports must have same length but ninput_items[0]=" +
            std::to_string(ninput_items[0]) + " and ninput_items[1]=" +
            std::to_string(ninput_items[1]));
      }
      int64_t min_inp = std::min(ninput_items[0], ninput_items[1]);
      if (min_inp < 1)
      {
        return 0; // no output
      }
      std::vector<gr::tag_t> tags1;
      get_tags_in_window(tags1, 0, 0, 1, pmt::string_to_symbol("packet_len"));
      if (tags1.size() == 0)
      {
        throw std::runtime_error(
            "[demapper_impl] ERROR: No packet_len tag found in input stream");
      }
      d_last_pkt_len = pmt::to_long(tags1[0].value);
      if (min_inp < d_last_pkt_len)
      {
        return 0; // Need more input
      }

      ////////////////////////////////////
      // 1) Process tags
      ////////////////////////////////////

      // Check if we have valid control
      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_ctrl_ok"));
      if (tags.size() == 0)
      {
        throw std::runtime_error(
            "[demapper_impl] ERROR: No rx_ctrl_ok tag found in input stream");
      }
      d_last_ctrl_ok = pmt::to_bool(tags[0].value);

      NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_ctrl_ok=" << d_last_ctrl_ok);

      // Retrive the rest of the tags
      d_found_rx_checksum = false;
      d_rx_data_checksum = 0;
      uint64_t abs_in_start = nitems_read(0);
      get_tags_in_range(tags, 0, abs_in_start, abs_in_start + 1);
      for (size_t i = 0; i < tags.size(); i++)
      {
        const auto &tg = tags[i];
        std::string key = pmt::symbol_to_string(tg.key);
        if (key == "rx_modtype")
        {
          d_last_modtype = pmt::to_uint64(tg.value);
          modtype_bits_to_index(d_last_modtype);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_modtype=" << d_last_modtype);
                    
        } 
        else if (key == "rx_current_phase")
        {
          d_current_phase = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_current_phase=" << d_current_phase);
        }
        else if (key == "rx_modtype_phase1")
        {
          d_last_modtype_phase1 = pmt::to_uint64(tg.value);
          modtype_bits_to_index(d_last_modtype_phase1);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_modtype_phase1=" << d_last_modtype_phase1);
        }
        else if (key == "rx_modtype_phase2")
        {
          d_last_modtype_phase2 = pmt::to_uint64(tg.value);
          modtype_bits_to_index(d_last_modtype_phase2);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_modtype_phase2=" << d_last_modtype_phase2);
        }
        else if (key == "rx_modtype_phase3")
        {
          d_last_modtype_phase3 = pmt::to_uint64(tg.value);
          modtype_bits_to_index(d_last_modtype_phase3);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_modtype_phase3=" << d_last_modtype_phase3);
        }
        else if (key == "rx_raw_ctrl")
        {
          const unsigned __int128 *raw_ctrl_ptr = reinterpret_cast<const unsigned __int128 *>(pmt::blob_data(tg.value));
          d_last_raw_ctrl = *raw_ctrl_ptr;
        }
        else if (key == "rx_nstrm")
        {
          d_last_nstrm = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_nstrm=" << d_last_nstrm);
        }
        else if (key == "rx_coding_rate")
        {
          d_last_coding_rate = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_coding_rate=" << d_last_coding_rate);
        }
        else if (key == "rx_data_checksum")
        {
          d_found_rx_checksum = true;
          d_rx_data_checksum = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_data_checksum=" << d_rx_data_checksum);
        }
        else if (key == "rx_syms_per_stream")
        {
          d_last_syms_per_stream = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_syms_per_stream=" << d_last_syms_per_stream);
        }
        else if (key == "rx_seqno")
        {
          d_last_seq_no = pmt::to_uint64(tg.value);
          NCJT_LOG(d_debug, "\t(" << cc << ")] Found rx_seqno=" << d_last_seq_no);
        }
        else if (key == "snr_rbs_db")
        {
          std::vector<float> snr_values = pmt::f32vector_elements(tg.value);
          size_t num_snr_elements = snr_values.size();
          d_snr_rbs_db.clear();
          d_snr_rbs_db.resize(num_snr_elements);
          std::memcpy(d_snr_rbs_db.data(), snr_values.data(), num_snr_elements * sizeof(float));
        }
      }
      consume_each(min_inp);

      //----------------------------------------------------------------------
      // 2) Hard-decision demapping of all d_last_pkt_len QAM symbols
      //----------------------------------------------------------------------
      // The mapper interleaves symbols across streams. So if d_last_nstrm>1,
      // we do the reverse interleaving. We'll reconstruct the bits the same
      // way the mapper did it.

      const gr_complex *in_data = static_cast<const gr_complex *>(input_items[0]);
      const gr_complex *in_csi = static_cast<const gr_complex *>(input_items[1]);

      d_coded_len = d_last_pkt_len * d_last_modtype; // total bits
      int d_phase2_coded_len = d_last_nstrm * (RG_NUM_OFDM_SYM[d_rgmode] * RG_NUM_DATA_SC[d_rgmode] - 64) * d_last_modtype_phase2;
      d_coded_buf = (uint8_t *)malloc(d_coded_len);

      int syms_per_stream = d_last_pkt_len / d_last_nstrm;
      for (int strm = 0; strm < d_last_nstrm; strm++)
      {
        for (int di = 0; di < syms_per_stream; di++)
        {
          int sym_idx = di * d_last_nstrm + strm;
          float x = in_data[sym_idx].real();
          float y = in_data[sym_idx].imag();
          float csi_val = in_csi[sym_idx].real(); // take the real part as amplitude

          // Hard-decision demap
          uint8_t bits[8]; // up to 8 bits
          gr::ncjt::demap_symbol(x, y, csi_val, d_last_modtype, bits);
          int out_sym_offset = di * (d_last_nstrm * d_last_modtype);
          for (int b = 0; b < d_last_modtype; b++)
          {
            d_coded_buf[out_sym_offset + b * d_last_nstrm + strm] = bits[b];
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////
      // 3) LDPC decode
      ///////////////////////////////////////////////////////////////////////
      if (d_last_coding_rate > 0)
      {
        double R = code_rates[d_last_coding_rate];
        int message_len_bits = int(std::floor(d_phase2_coded_len * R));
        message_len_bits = message_len_bits + (d_last_modtype_phase2 - (message_len_bits % d_last_modtype));
        //
        std::vector<srsran::log_likelihood_ratio> llrs(d_phase2_coded_len);
        for (int i = 0; i < d_phase2_coded_len; i++)
        {
          llrs[i] = (d_coded_buf[i] == 0) ? +10 : -10;
        }
        //
        int seg_out_size = d_last_syms_per_stream;
        int num_segs = d_phase2_coded_len / seg_out_size;
        int base_in_bits_per_seg = message_len_bits / num_segs;
        int remainder = message_len_bits % num_segs;
        d_info_buf = (uint8_t *)malloc(message_len_bits);
        d_info_len = 0;

        int start_idx = 0;
        for (int seg = 0; seg < num_segs; seg++)
        {
          int in_bits_per_seg =
              base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
          NCJT_LOG(d_debug, "\t\t >> Segment " << seg
                    << " (start_idx=" << start_idx
                    << ", in_bits_per_seg=" << in_bits_per_seg << ")");
          auto d_bits = gr::ncjt::ldpc_decode(in_bits_per_seg,
                                              std::vector<srsran::log_likelihood_ratio>(
                                                  llrs.begin() + start_idx,
                                                  llrs.begin() + start_idx + seg_out_size),
                                              d_ldpc_decoder.get(),
                                              d_ldpc_dematcher.get());
          std::memcpy(d_info_buf + d_info_len, d_bits.data(), d_bits.size());
          d_info_len += d_bits.size();
          start_idx += seg_out_size;
        }
      }
      else
      {
        d_info_buf = d_coded_buf;
        d_info_len = d_phase2_coded_len;
      }

      ///////////////////////////////////////////////////////////////////////
      // 4) CRC16 checksum
      ///////////////////////////////////////////////////////////////////////
      if (d_found_rx_checksum)
      {
        uint16_t computed_crc = gr::ncjt::compute_crc16(d_info_buf, d_info_len);
        bool crc_ok = (computed_crc == (uint16_t)d_rx_data_checksum);
        NCJT_LOG(d_debug, "\t(" << cc << ")] Computed CRC16=0x"
                  << std::hex << computed_crc
                  << "  vs. 0x" << d_rx_data_checksum << std::dec
                  << " => match=" << crc_ok);
        auto d_name = pmt::string_to_symbol(this->name());
        for (int op = 0; op < 1 + int(d_output_raw); op++)
        {
          // Insert a "rx_data_crc_passed" tag indicating pass/fail
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_data_crc"),
                       pmt::from_bool(crc_ok),
                       d_name);
          // Even if the CRC failed, we still output these tags:
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_ctrl_ok"),
                       pmt::from_bool(d_last_ctrl_ok),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_seqno"),
                       pmt::from_uint64(d_last_seq_no),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_modtype"),
                       pmt::from_uint64(d_last_modtype),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_current_phase"),
                       pmt::from_uint64(d_current_phase),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_modtype_phase1"),
                       pmt::from_uint64(d_last_modtype_phase1),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_modtype_phase2"),
                       pmt::from_uint64(d_last_modtype_phase2),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_modtype_phase3"),
                       pmt::from_uint64(d_last_modtype_phase3),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_raw_ctrl"),
                       pmt::make_blob(&d_last_raw_ctrl, sizeof(d_last_raw_ctrl)),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_nstrm"),
                       pmt::from_uint64(d_last_nstrm),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_coding_rate"),
                       pmt::from_uint64(d_last_coding_rate),
                       d_name);
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("rx_data_checksum"),
                       pmt::from_uint64(d_rx_data_checksum),
                       d_name);
          ////
          add_item_tag(op, nitems_written(op), pmt::string_to_symbol("snr_rbs_db"),
                       pmt::init_f32vector(d_snr_rbs_db.size(), d_snr_rbs_db.data()),
                       d_name);
        }
      }

      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("packet_len"),
                   pmt::from_long(d_info_len / d_last_modtype_phase2),
                   pmt::string_to_symbol(this->name()));

      assert(d_info_len % d_last_modtype_phase2 == 0);

      if (d_output_raw)
      {
        add_item_tag(1, nitems_written(1), pmt::string_to_symbol("packet_len"),
                     pmt::from_long(d_phase2_coded_len / d_last_modtype_phase2),
                     pmt::string_to_symbol(this->name()));
      }

      assert(d_phase2_coded_len % d_last_modtype_phase2 == 0);

      // ----------------------------------------------------------------
      // 5) If deterministic input => measure coded BER and uncoded BER
      // ----------------------------------------------------------------
      if (d_deterministic_input)
      {
        // Recalculate how many *info bits* the mapper used
        int frame_data_syms = d_last_pkt_len * d_last_nstrm;
        // int frame_data_bits_out = frame_data_syms * d_last_modtype;
        int frame_data_bits_phase2_out = frame_data_syms * d_last_modtype_phase2;
        double R = code_rates[d_last_coding_rate];
        int in_bits_needed = frame_data_bits_phase2_out;
        if (d_last_coding_rate > 0)
        {
          in_bits_needed = int(std::floor(frame_data_bits_phase2_out * R));
          in_bits_needed = in_bits_needed + (d_last_modtype_phase2 - (in_bits_needed % d_last_modtype_phase2));
        }
        // Generate the same PRNG bits used by mapper_muxer_impl
        std::mt19937 gen(d_last_seq_no);
        std::uniform_int_distribution<int> dist(0, 1);
        std::vector<uint8_t> reference_bits(in_bits_needed);
        for (int i = 0; i < in_bits_needed; i++)
        {
          reference_bits[i] = dist(gen) & 0x1;
        }
        // ---------------------------
        // 5a) Coded BER
        //     Compare decoded info bits vs reference_bits
        // ---------------------------
        int compare_len = std::min(d_info_len, in_bits_needed);
        int bit_errors = 0;
        for (int i = 0; i < compare_len; i++)
        {
          if (d_info_buf[i] != reference_bits[i])
          {
            bit_errors++;
          }
        }
        double coded_ber = (compare_len > 0)
                               ? double(bit_errors) / double(compare_len)
                               : 0.0;
        // ---------------------------
        // 5b) Uncoded BER
        //     Re-encode the reference_bits so we get the same coded bits
        //     that the TX used, then compare to d_coded_buf
        // ---------------------------
        std::vector<uint8_t> reference_coded_bits;
        if (d_last_coding_rate > 0)
        {
          int seg_out_size = d_last_syms_per_stream;
          int num_segs = frame_data_bits_phase2_out / seg_out_size;
          int base_in_bits_per_seg = in_bits_needed / num_segs;
          int remainder = in_bits_needed % num_segs;
          reference_coded_bits.reserve(d_phase2_coded_len);
          int start_idx = 0;
          for (int seg = 0; seg < num_segs; seg++)
          {
            int in_bits_per_seg =
                base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
            auto seg_encoded = gr::ncjt::ldpc_encode(
                std::vector<uint8_t>(reference_bits.begin() + start_idx,
                                     reference_bits.begin() + start_idx + in_bits_per_seg),
                seg_out_size,
                d_ldpc_encoder.get(),
                d_ldpc_matcher.get());
            reference_coded_bits.insert(reference_coded_bits.end(),
                                        seg_encoded.begin(), seg_encoded.end());
            start_idx += in_bits_per_seg;
          }
          std::cout << "### "
                    << "reference_bits.size(): " << reference_bits.size()
                    << ", reference_coded_bits.size(): " << reference_coded_bits.size() 
                    << ", d_phase2_coded_len: " << d_phase2_coded_len
                    << ", d_coded_len: " << d_coded_len 
                    << std::endl;
          assert(reference_coded_bits.size() == (size_t)d_phase2_coded_len);
        }
        else
        {
          // no coding => coded bits = raw bits
          reference_coded_bits = reference_bits;
        }
        int uncoded_compare_len = std::min((int)reference_coded_bits.size(), d_phase2_coded_len);
        int uncoded_errors = 0;
        for (int i = 0; i < uncoded_compare_len; i++)
        {
          if (reference_coded_bits[i] != d_coded_buf[i])
          {
            uncoded_errors++;
          }
        }
        double uncoded_ber = (uncoded_compare_len > 0)
                                 ? double(uncoded_errors) / double(uncoded_compare_len)
                                 : 0.0;
        NCJT_LOG(d_debug, "\t(" << cc << ")] => coded_ber=" << coded_ber
                  << ", uncoded_ber=" << uncoded_ber);
        // Add tags for coded and uncoded BER
        for (int op = 0; op < 1 + int(d_output_raw); op++)
        {
          add_item_tag(op, nitems_written(op),
                       pmt::string_to_symbol("rx_coded_ber"),
                       pmt::from_double(coded_ber),
                       pmt::string_to_symbol(this->name()));
          add_item_tag(op, nitems_written(op),
                       pmt::string_to_symbol("rx_uncoded_ber"),
                       pmt::from_double(uncoded_ber),
                       pmt::string_to_symbol(this->name()));
        }
      }

      // Pack d_last_modtype_phase2 bits into each byte (decoded)
      int out_info_bytes = d_info_len / d_last_modtype_phase2;
      uint8_t *out_dec = static_cast<uint8_t *>(output_items[0]);
      for (int i = 0; i < out_info_bytes; i++)
      {
        uint8_t val = 0;
        for (int b = 0; b < d_last_modtype_phase2; b++)
        {
          val = (val << 1) | d_info_buf[i * d_last_modtype_phase2 + b];
        }
        out_dec[i] = val;
      }
      std::cout << "AAA " << out_info_bytes << std::endl;
      produce(0, out_info_bytes);
      // Pack d_last_modtype_phase2 bits into each byte (uncoded)
      if (d_output_raw)
      {
        uint8_t *out_unc = static_cast<uint8_t *>(output_items[1]);
        int out_coded_bytes = d_phase2_coded_len / d_last_modtype_phase2;
        for (int i = 0; i < out_coded_bytes; i++)
        {
          uint8_t val = 0;
          for (int b = 0; b < d_last_modtype_phase2; b++)
          {
            val = (val << 1) | d_coded_buf[i * d_last_modtype_phase2 + b];
          }
          out_unc[i] = val;
        }
        std::cout << "BBB " << out_coded_bytes << std::endl;
        produce(1, out_coded_bytes);
      }
      if (d_last_coding_rate > 0)
      {
        free(d_info_buf);
        free(d_coded_buf);
        d_info_buf = nullptr;
        d_coded_buf = nullptr;
        d_info_len = 0;
        d_coded_len = 0;
      }
      else
      {
        free(d_info_buf);
        d_info_buf = nullptr;
        d_coded_buf = nullptr;
        d_info_len = 0;
        d_coded_len = 0;
      }

      return WORK_CALLED_PRODUCE;
    }

  } /* namespace ncjt */
} /* namespace gr */
