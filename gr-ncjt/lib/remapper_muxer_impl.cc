/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "remapper_muxer_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/ncjt/rg_modes.h>
#include <random>
#include "common.h"

namespace gr
{
  namespace ncjt
  {
    // ------------------------------------------------------------------------
    // Factory
    // ------------------------------------------------------------------------
    remapper_muxer::sptr
    remapper_muxer::make(int rgmode, int nstrm, bool reencode, bool debug)
    {
      return gnuradio::make_block_sptr<remapper_muxer_impl>(
          rgmode,
          nstrm,
          reencode,
          debug);
    }

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    remapper_muxer_impl::remapper_muxer_impl(int rgmode,
                                             int nstrm,
                                             bool reencode,
                                             bool debug)
        : gr::tagged_stream_block(
              "remapper_muxer",
              gr::io_signature::make(1, 1, sizeof(uint8_t)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              "packet_len"),
          cc(0),
          d_phase(-1), // Phase is set later based on input tags
          d_rgmode(rgmode),
          d_nstrm(nstrm),
          d_modtype(-1),
          d_code_rate(-1),
          d_reencode(reencode),
          d_debug(debug),
          d_seqno(0)
    {

      NCJT_LOG(d_debug, " rgmode=" << rgmode << ", nstrm=" << d_nstrm 
               << ", reencode=" << d_reencode);

      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else
      {
        d_n_ofdm_syms = RG_NUM_OFDM_SYM[rgmode];
        d_sd_num = RG_NUM_DATA_SC[rgmode];
      }

      // Constellations
      d_constellation_qpsk = &CONST_QPSK;
      d_constellation_16qam = &CONST_16QAM;
      d_constellation_64qam = &CONST_64QAM;
      d_constellation_256qam = &CONST_256QAM;

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
    }

    /*
     * Our virtual destructor.
     */
    remapper_muxer_impl::~remapper_muxer_impl()
    {
    }

    int remapper_muxer_impl::calculate_output_stream_length(
        const gr_vector_int &ninput_items)
    {
      int noutput_items = d_n_ofdm_syms * d_sd_num * d_nstrm; // 64 QPSK symbols + 2016 QAM symbols
      NCJT_LOG(d_debug, "calculate_output_stream_length: "
                            << "ninput_items[0]=" << ninput_items[0]
                            << ", noutput_items=" << noutput_items);
      return noutput_items;
    }

    int remapper_muxer_impl::work(int noutput_items,
                                  gr_vector_int &ninput_items,
                                  gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items)
    {
      cc++;
      gr_complex *out0 = static_cast<gr_complex *>(output_items[0]);


      // Get rx_seqno tag
      std::vector<gr::tag_t> tags;
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_seqno"));
      d_seqno = pmt::to_uint64(tags[0].value);

      NCJT_LOG(d_debug, "(" << cc << ") -------- "
                            << "\n\tCalled, noutput_items=" << noutput_items
                            << ", ninput_items[0]=" << ninput_items[0]
                            << ", d_seqno=" << d_seqno);

      // 1) Obtain the tags

      // Get rx_modtype_phase1, phase2, and phase3 tags
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

      // Get rx_current_phase tag
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_current_phase"));
      if (!tags.size())
        throw std::runtime_error(
            "[remapper_muxer_impl] ERROR: The rx_current_phase tag was not found in input stream");
      d_phase = pmt::to_uint64(tags[0].value) + 1;

      // get rx_coding_rate tag
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_coding_rate"));
      if (!tags.size())
        throw std::runtime_error(
            "[remapper_muxer_impl] ERROR: The rx_coding_rate tag was not found in input stream");
      d_code_rate = pmt::to_uint64(tags[0].value);

      NCJT_LOG(d_debug, "d_phase=" << d_phase);

      if (d_phase == 2)
      {
        d_modtype = phase2_modtype;
      }
      else if (d_phase == 3)
      {
        d_modtype = phase3_modtype;
      }
      else
      {
        throw std::runtime_error(
            "[remapper_muxer_impl] ERROR: d_phase must be 2 or 3, but got " + std::to_string(d_phase));
      }
      ///////

      // 2) Prepare the CTRL and obtain its symbols

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_raw_ctrl"));
      if (!tags.size())
        throw std::runtime_error("[remapper_muxer_impl] ERROR: No rx_raw_ctrl tag found in input stream");
      const unsigned __int128 *raw_ctrl = reinterpret_cast<const unsigned __int128 *>(pmt::blob_data(tags[0].value));

      CTRL ctrl(d_debug);
      ctrl.set_raw(*raw_ctrl);

      if (d_phase == 3)
      {
        // Write SNRs (dB) for each RB into extended field
        get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("snr_rbs_db"));
        std::vector<float> snr_values = pmt::f32vector_elements(tags[0].value);
        size_t num_snr_elements = snr_values.size();

        // print SNRs for debugging
        if (d_debug)
        {
          std::cout << "[remapper_muxer_impl] SNRs for RG mode "
                    << d_rgmode << ": ";
          for (size_t i = 0; i < num_snr_elements; i++)
          {
            std::cout << snr_values[i] << " dB";
            if (i < num_snr_elements - 1)
              std::cout << ", ";
          }
          std::cout << std::endl;
        }

        uint64_t packed = quantize_snrs(snr_values.data(), num_snr_elements,
                                        RB_B0[d_rgmode], RB_Bd[d_rgmode],
                                        MIN_ABS, MAX_ABS,
                                        MIN_DIFF, MAX_DIFF);
        // std::cout << "[remapper_muxer_impl] EXTENDED: " << std::hex << packed << std::dec << std::endl;
        ctrl.set_extended(packed);
      }

      ctrl.set_seq_number(d_seqno); // Do not remove!

      // ctrl.set_data_checksum(data_crc16);

      std::vector<gr_complex> ctrl_syms;
      if (d_phase == 3)
      {
        ctrl_syms = ctrl.pack_and_modulate_16qam();
      }
      else
      {
        ctrl_syms = ctrl.pack_and_modulate_qpsk();
      }

      // 3) Place CTRL symbols in the output buffer
      for (int i = 0; i < d_nstrm; i++)
      {
        for (int j = 0; j < 64; j++)
        {
          out0[j * d_nstrm + i] = ctrl_syms[j];
        }
      }

      ///////
      std::vector<uint8_t> raw_in_bits; // bits to feed into LDPC or direct
      // Unpack bits from each input byte into phase2_modtype bits
      int in_bits_needed = ninput_items[0] * phase2_modtype;
      raw_in_bits.resize(in_bits_needed);
      for (int i = 0; i < ninput_items[0]; i++)
      {
        uint8_t val = static_cast<const uint8_t *>(input_items[0])[i];
        for (int b = 0; b < phase2_modtype; b++)
        {
          int shift = (phase2_modtype - 1) - b;
          raw_in_bits[i * phase2_modtype + b] = (val >> shift) & 0x1;
        }
      }

      NCJT_LOG(d_debug, "(" << cc << ") raw_in_bits.size()=" << raw_in_bits.size()
                            << ", in_bits_needed=" << in_bits_needed
                            << ", phase2_modtype=" << phase2_modtype);
      ///////

      // How many data symbols per frame?
      int frame_data_syms = d_nstrm * (d_n_ofdm_syms * d_sd_num - 64);

      // Data portion offset in the output (in symbols)
      int data_sym_offset = d_nstrm * 64;

      int num_periods = frame_data_syms / d_nstrm; // each period is one symbol from each stream

      int frame_data_bits_out_capacity = frame_data_syms * d_modtype;
      int frame_data_bits_phase2_out = frame_data_syms * phase2_modtype;

      //--------------------------------------------------------------------
      // LDPC encode (or pass through) the raw_in_bits so we get 'frame_data_bits_out' bits
      //--------------------------------------------------------------------

      std::vector<uint8_t> used_bits;

      used_bits.clear();
      used_bits.reserve(frame_data_bits_out_capacity);

      if (d_code_rate == 0 || !d_reencode)
      {
        // No coding, just copy the needed bits
        used_bits.insert(used_bits.end(), raw_in_bits.begin(),
                         raw_in_bits.begin() + frame_data_bits_phase2_out);
      }
      else
      {
        ////// New segmented processing
        int seg_out_size = d_n_ofdm_syms * d_sd_num - 64;
        int num_segs = frame_data_bits_phase2_out / seg_out_size;
        int base_in_bits_per_seg = in_bits_needed / num_segs;
        int remainder = in_bits_needed % num_segs;
        NCJT_LOG(d_debug,
                 " (" << cc << ")"
                      << "\n\t Segmented processing: "
                      << "\n\t in_bits_needed=" << in_bits_needed
                      << "\n\t frame_data_bits_phase2_out=" << frame_data_bits_phase2_out
                      << "\n\t seg_out_size=" << seg_out_size
                      << "\n\t num_segs=" << num_segs
                      << "\n\t base_in_bits_per_seg=" << base_in_bits_per_seg
                      << "\n\t remainder=" << remainder);

        int start_idx = 0;
        for (int seg = 0; seg < num_segs; seg++)
        {
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

      NCJT_LOG(d_debug, "Length before padding: " << used_bits.size());
      // Padding: Fill the remaining frame_data_bits_out_capacity with random bits
      std::mt19937 gen_padding(d_seqno + 11); // different seed for padding
      std::uniform_int_distribution<int> dist_padding(0, 1);
      while ((int)used_bits.size() < frame_data_bits_out_capacity)
      {
        used_bits.push_back(dist_padding(gen_padding) & 0x1);
      }

      NCJT_LOG(d_debug, "Length after padding: "
                            << used_bits.size()
                            << " (padded with " << (used_bits.size() - frame_data_bits_phase2_out) << " random bits)");

      assert(used_bits.size() == (size_t)frame_data_bits_out_capacity);

      // Map used_bits -> QAM symbols
      int tmp_num_syms_to_show = 3;
      int tmpflag = tmp_num_syms_to_show;
      for (int k = 0; k < num_periods; k++)
      {
        int period_offset = k * d_nstrm * d_modtype;
        for (int s = 0; s < d_nstrm; s++)
        {
          int val = 0;
          // gather bits for this symbol
          // if (tmpflag == tmp_num_syms_to_show && d_seqno == 0) {
          //   std::cout << "Remapper bits: ";
          // }
          for (int j = 0; j < d_modtype; j++)
          {
            int bit_index = period_offset + s + j * d_nstrm;
            // if (tmpflag > 0 && d_seqno == 0) {
            //   std::cout << (int)used_bits[bit_index] << " ";
            // }
            val |= (used_bits[bit_index] << j);
          }
          // if (tmpflag == 0 && d_seqno == 0) {
          //   std::cout << std::endl;
          //   tmpflag--;
          // } else if (tmpflag > 0 && d_seqno == 0) {
          //   std::cout << " - ";
          //   tmpflag--;
          // }
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

      int total_syms_out = d_nstrm * d_n_ofdm_syms * d_sd_num;
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("packet_len"),
                   pmt::from_long(total_syms_out));
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("seqno"),
                   pmt::from_long(d_seqno));

      d_bit_buffer.clear();

      d_seqno++;

      NCJT_LOG(d_debug, "(" << cc << ") Summary:"
                            << "\n\t d_phase=" << d_phase
                            << ", d_modtype=" << d_modtype
                            << ", d_code_rate=" << d_code_rate
                            << "\n\t total_syms_out=" << total_syms_out);

      return total_syms_out;
    }
  } /* namespace ncjt */
} /* namespace gr */
