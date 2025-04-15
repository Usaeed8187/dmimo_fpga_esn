/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mapper_muxer_phase3_impl.h"
#include <gnuradio/io_signature.h>
#include "common.h"

namespace gr
{
  namespace ncjt
  {

    // ------------------------------------------------------------------------
    // Factory
    // ------------------------------------------------------------------------
    mapper_muxer_phase3::sptr
    mapper_muxer_phase3::make(int nstrm, int n_ofdm_syms,
                              int sd_num, bool use_polar, bool debug)
    {
      return gnuradio::make_block_sptr<mapper_muxer_phase3_impl>(
          nstrm, n_ofdm_syms, sd_num, use_polar, debug);
    }

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    mapper_muxer_phase3_impl::mapper_muxer_phase3_impl(int nstrm,
                                                       int n_ofdm_syms,
                                                       int sd_num,
                                                       bool use_polar,
                                                       bool debug)
        : gr::tagged_stream_block(
              "mapper_muxer_phase3",
              gr::io_signature::make(1, 1, sizeof(uint8_t)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              "packet_len"),
          d_nstrm(nstrm), d_modtype(-1), d_n_ofdm_syms(n_ofdm_syms),
          d_sd_num(sd_num), d_code_rate(-1),
          d_debug(debug), d_seqno(0)
    {

      if (d_debug)
      {
        std::cout << "[CONSTRUCTOR mapper_muxer_phase3_impl (nstrm=" << nstrm
                  << ", n_ofdm_syms=" << n_ofdm_syms << ", sd_num=" << sd_num
                  << ", use_polar=" << use_polar << ", debug=" << debug << ")]"
                  << std::endl;
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
    mapper_muxer_phase3_impl::~mapper_muxer_phase3_impl() {}

    int mapper_muxer_phase3_impl::calculate_output_stream_length(
        const gr_vector_int &ninput_items)
    {
      int noutput_items = d_n_ofdm_syms * d_sd_num * d_nstrm; // 64 QPSK symbols + 2016 QAM symbols
      if (d_debug)
      {
        std::cout << "[mapper_muxer_phase3_impl::calculate_output_stream_length] "
                  << "ninput_items[0]=" << ninput_items[0] << ", noutput_items=" << noutput_items << std::endl;
      }
      return noutput_items;
    }

    int mapper_muxer_phase3_impl::work(int noutput_items,
                                       gr_vector_int &ninput_items,
                                       gr_vector_const_void_star &input_items,
                                       gr_vector_void_star &output_items)
    {
      cc++;
      if (d_debug)
      {
        std::cout << "[mapper_muxer_phase3_impl::work(" << cc
                  << ")] Called, noutput_items=" << noutput_items
                  << ", ninput_items[0]=" << ninput_items[0] << "" << std::endl;
      }
      gr_complex *out0 = static_cast<gr_complex *>(output_items[0]);

      std::vector<gr::tag_t> tags;
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_seqno"));
      d_seqno = pmt::to_uint64(tags[0].value);

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_modtype_phase3"));
      if (!tags.size())
        throw std::runtime_error("[mapper_muxer_phase3_impl] ERROR: No rx_modtype_phase3 tag found in input stream");
      d_modtype = pmt::to_uint64(tags[0].value);

      // How many data symbols per frame?
      int frame_data_syms = (d_nstrm * d_n_ofdm_syms * d_sd_num) - (d_nstrm * 64);

      // The final number of bits to be mapped onto data symbols:
      int frame_data_bits_out = frame_data_syms * d_modtype;

      // Fill d_bit_buffer with zeros
      d_bit_buffer.resize(frame_data_bits_out, 0);

      // Unpack bits from each input byte into d_modtype bits
      int nbytes_in = frame_data_bits_out / d_modtype;
      for (int i = 0; i < nbytes_in; i++)
      {
        uint8_t val = static_cast<const uint8_t *>(input_items[0])[i];
        for (int b = 0; b < d_modtype; b++)
        {
          int shift = (d_modtype - 1) - b;
          d_bit_buffer[i * d_modtype + b] = (val >> shift) & 0x1;
        }
      }

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_raw_ctrl"));
      if (!tags.size())
        throw std::runtime_error("[mapper_muxer_phase3_impl] ERROR: No rx_raw_ctrl tag found in input stream");
      const unsigned __int128 *raw_ctrl = reinterpret_cast<const unsigned __int128 *>(pmt::blob_data(tags[0].value));

      CTRL ctrl(d_debug);
      ctrl.set_raw(*raw_ctrl);

      // Write SNRs (dB) for each RB into extended field

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("snr_rbs_db"));
      std::vector<float> snr_values = pmt::f32vector_elements(tags[0].value);
      size_t num_snr_elements = snr_values.size();

      uint64_t packed = quantize_snrs(snr_values.data(), num_snr_elements,
                                      B0, Bd,
                                      MIN_ABS, MAX_ABS,
                                      MIN_DIFF, MAX_DIFF);
      // std::cout << "[mapper_muxer_phase3_impl] EXTENDED: " << std::hex << packed << std::dec << std::endl;
      ctrl.set_extended(packed);

      ctrl.set_seq_number(d_seqno); // Do not remove!

      // ctrl.set_data_checksum(data_crc16);
      std::vector<gr_complex> ctrl_syms = ctrl.pack_and_modulate_16qam();

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

      // The number of "OFDM symbol periods" in the data portion
      int num_periods = frame_data_syms / d_nstrm;

      std::vector<uint8_t> used_bits(d_bit_buffer.begin(),
                                     d_bit_buffer.begin() + frame_data_bits_out);

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

      int total_syms_out = d_nstrm * d_n_ofdm_syms * d_sd_num;
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("packet_len"),
                   pmt::from_long(total_syms_out));
      add_item_tag(0, nitems_written(0), pmt::string_to_symbol("seqno"),
                   pmt::from_long(d_seqno));

      d_bit_buffer.clear();

      // Bump sequence number (to be overridden by tags if valid control packet)
      d_seqno++;

      return total_syms_out;
    }

  } /* namespace ncjt */
} /* namespace gr */
