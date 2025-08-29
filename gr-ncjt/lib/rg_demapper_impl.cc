/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "rg_demapper_impl.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <gnuradio/io_signature.h>
#include <iomanip> // for std::hex, std::dec
#include <iostream>
#include <pmt/pmt.h>
#include <stdexcept>
#include <gnuradio/ncjt/rg_modes.h>
#include "common.h"
#include <limits>

namespace gr
{
  namespace ncjt
  {

    // ------------------------------------------------------------------------
    // Factory
    // ------------------------------------------------------------------------
    rg_demapper::sptr rg_demapper::make(int phase,
                                        int rgmode,
                                        int nstrm,
                                        bool usecsi,
                                        bool debug)
    {
      return gnuradio::make_block_sptr<rg_demapper_impl>(
          phase, rgmode, nstrm, usecsi, debug);
    }

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    rg_demapper_impl::rg_demapper_impl(int phase,
                                       int rgmode,
                                       int nstrm,
                                       bool usecsi,
                                       bool debug)
        : gr::tagged_stream_block("rg_demapper",
                                  gr::io_signature::make(usecsi ? nstrm : nstrm,
                                                         usecsi ? 2 * nstrm : nstrm,
                                                         sizeof(gr_complex)),
                                  gr::io_signature::make(2, 2, sizeof(gr_complex)),
                                  "packet_len"),
          d_phase(phase),
          d_rgmode(rgmode),
          d_ctrl_ok(false),
          d_nstrm_param(nstrm),
          d_nstrm_phase1(nstrm),
          d_nstrm_phase2(nstrm),
          d_nstrm_phase3(nstrm),
          d_data_checksum(0),
          d_seqno(-1),
          d_debug(debug),
          cc(0),
          d_usecsi(usecsi),
          d_ctrl_obj(false), // TODO: pass debug flag
          d_extended(0),
          d_raw_ctrl(0)
    {
      cc = 0;
      d_seqno = -1;
      d_data_checksum = 0;
      d_ctrl_ok = false;
      if (d_nstrm_param < 1 || d_nstrm_param > 8)
      {
        throw std::runtime_error("[rg_demapper] invalid nstrm_param (1..8).");
      }
      if (rgmode < 0 || rgmode >= 8)
          throw std::runtime_error("[rg_demapper] Unsupported RG mode");
      else {
        d_modtype = 2;
        d_modtype_phase1 = 2;
        d_modtype_phase2 = 2;
        d_modtype_phase3 = 2;
        //
        d_code_rate = 0;
        d_code_rate_phase1 = 0;
        d_code_rate_phase2 = 0;
        d_code_rate_phase3 = 0;
        //
        d_n_ofdm_syms = RG_NUM_OFDM_SYM[rgmode];
        d_sd_num = RG_NUM_DATA_SC[rgmode];
        d_fftsize = RG_FFT_SIZE[rgmode];
      }
      modtype_bits_to_index(d_modtype);

      // NCJT_LOG(d_debug,
      //           "\n\tphase=" << d_phase
      //           << "\n\trgmode=" << d_rgmode
      //           << "\n\tnstrm_param=" << d_nstrm_param
      //           << "\n\tusecsi=" << d_usecsi);

      // int max_output_items = (d_n_ofdm_syms * d_sd_num - 64) * d_nstrm_param;
      // set_min_output_buffer(0, max_output_items); // Output port 0: demapped symbols
      // set_min_output_buffer(1, max_output_items); // Output port 1: CSI

      // NCJT_LOG(d_debug,
      //         "Set minimum output buffer size to " << max_output_items
      //         << " items for RG mode " << rgmode
      //         << " with " << d_nstrm_param << " streams");

      const size_t frame_syms =
      static_cast<size_t>(d_nstrm_param) *
      static_cast<size_t>(d_n_ofdm_syms * d_sd_num - 64);

      /* Make sure we are only called with full–frame multiples          */
      set_output_multiple(frame_syms);

      /* Enlarge the buffers on both output ports so one frame fits       */
      set_min_output_buffer(0, frame_syms);   // data
      set_min_output_buffer(1, frame_syms);   // CSI (or 1 + 0 j if !usecsi)

      NCJT_LOG(d_debug,
                "rg_demapper_impl: phase=" << d_phase
                << ", rgmode=" << d_rgmode
                << ", nstrm_param=" << d_nstrm_param
                << ", usecsi=" << d_usecsi
                << ", sd_num=" << d_sd_num
                << ", n_ofdm_syms=" << d_n_ofdm_syms
                << ", fftsize=" << d_fftsize
                << ", frame_syms=" << frame_syms);

      set_tag_propagation_policy(gr::block::TPP_DONT);
    }

    // ------------------------------------------------------------------------
    // TSB
    // ------------------------------------------------------------------------
    int rg_demapper_impl::calculate_output_stream_length(
        const gr_vector_int &ninput_items)
    {
      if (ninput_items[0] < 64)
      {
        throw std::runtime_error(
            "[rg_demapper] ERROR: input ports must have at least 64 items");
      }
      for (int p = 1; p < d_nstrm_param; p++) {
        if (ninput_items[p] != ninput_items[0]) {
          throw std::runtime_error("[rg_demapper] ERROR: input ports must have same length");
        }
      }

      int total_data_syms = (ninput_items[0] - 64) * d_nstrm_param;

      NCJT_LOG(d_debug, "(" << cc << ")"
                << "\n\tninput_items[0]=" << ninput_items[0]
                << "\n\tnstrm_param=" << d_nstrm_param
                << "\n\tTotal data symbols: " << total_data_syms);
      return total_data_syms;
    }

    void rg_demapper_impl::update_seqno()
    {
      d_seqno = d_ctrl_obj.get_seq_number();
    }

    void rg_demapper_impl::update_reserved()
    {
      d_reserved = d_ctrl_obj.get_reserved();
    }

    void rg_demapper_impl::update_coding_rate()
    {
      d_code_rate_phase1 = d_ctrl_obj.get_coding_rate_phase1();
      d_code_rate_phase2 = d_ctrl_obj.get_coding_rate_phase2();
      d_code_rate_phase3 = d_ctrl_obj.get_coding_rate_phase3();
      switch (d_phase)
      {
      case 1:
        d_code_rate = d_code_rate_phase1;
        break;
      case 2:
        d_code_rate = d_code_rate_phase2;
        break;
      case 3:
        d_code_rate = d_code_rate_phase3;
        break;
      }
    }

    void rg_demapper_impl::update_modtype()
    {
      d_modtype_phase1 = modtype_index_to_bits(d_ctrl_obj.get_mod_type_phase1());
      d_modtype_phase2 = modtype_index_to_bits(d_ctrl_obj.get_mod_type_phase2());
      d_modtype_phase3 = modtype_index_to_bits(d_ctrl_obj.get_mod_type_phase3());
      switch (d_phase)
      {
      case 1:
        d_modtype = d_modtype_phase1;
        break;
      case 2:
        d_modtype = d_modtype_phase2;
        break;
      case 3:
        d_modtype = d_modtype_phase3;
        break;
      }
    }

    void rg_demapper_impl::update_data_checksum()
    {
      d_data_checksum = d_ctrl_obj.get_data_checksum();
    }

    // ------------------------------------------------------------------------
    // main TSB work
    // ------------------------------------------------------------------------
    int rg_demapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
    {
      cc++;
      int ctrl_syms_len = 64;
      int nports = d_usecsi ? 2 * d_nstrm_param : d_nstrm_param;

      NCJT_LOG(d_debug, "(" << cc << ")"
                << "\n\tnoutput_items=" << noutput_items
                << "\n\tninput_items.size()=" << ninput_items.size()
                << "\n\tninput_items[0]=" << ninput_items[0]);

      int min_in = ninput_items[0];
      for (int p = 0; p < nports; p++)
      {
        if (ninput_items[p] < min_in)
        {
          min_in = ninput_items[p];
        }
      }
      // All input streams must have the same number of items
      for (int p = 0; p < nports; p++)
      {
        if (ninput_items[p] != min_in)
        {
          throw std::runtime_error("[rg_demapper] ERROR: input ports must have same length");
        }
      }
      if (min_in == 0)
      {
        NCJT_LOG(d_debug, "(" << cc << ") No data, return 0");
        return 0; // no data
      }

      // Parse the control symbols if enabled
      d_seqno++;
      if (true)
      {
        d_ctrl_ok = false;

        for (int s = 0; s < d_nstrm_param; s++)
        {
          const gr_complex *in_sym = static_cast<const gr_complex *>(input_items[s]);
          std::vector<gr_complex> ctrl_syms(in_sym, in_sym + 64);
          const gr_complex *in_csi = static_cast<const gr_complex *>(input_items[d_nstrm_param + s]);
          std::vector<gr_complex> ctrl_csi;
          if (d_usecsi)
          {
            ctrl_csi.assign(in_csi, in_csi + 64);
          }
          else
          {
            ctrl_csi.resize(64, gr_complex(1.0f, 0.0f));
          }
          NCJT_LOG(d_debug, "\t(" << cc << ")] Demodulating control symbols on stream " << s);
          if (d_phase == 1)
            d_ctrl_ok = d_ctrl_obj.demodulate_and_unpack_qpsk(ctrl_syms, ctrl_csi);
          else if (d_phase == 2)
            d_ctrl_ok = d_ctrl_obj.demodulate_and_unpack_qpsk(ctrl_syms, ctrl_csi);
          else if (d_phase == 3)
            d_ctrl_ok = d_ctrl_obj.demodulate_and_unpack_16qam(ctrl_syms, ctrl_csi);
          else
            throw std::runtime_error(
                "[rg_demapper] ERROR: Invalid phase (currently supports phase 2 with normal CTRL and phase 3 with extended CTRL)");
          if (!d_ctrl_ok)
          {
            NCJT_LOG(d_debug, "(" << cc << ") Failed to demodulate control symbols on stream " << s);
            continue;
          }

          NCJT_LOG(d_debug, "(" << cc << ") Successfully demodulated control symbols on stream " << s);

          update_seqno();
          update_reserved();
          // nstrm is not obtained from CTRL for now. Current design of upstream block does not allow this. @TODO
          update_coding_rate();
          update_modtype();
          update_data_checksum();

          d_extended = d_ctrl_obj.get_extended();
          d_raw_ctrl = d_ctrl_obj.get_raw();
        }
      }

      // Add tags
      auto d_wrt = nitems_written(0);
      auto d_name = pmt::string_to_symbol(this->name());
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_ctrl_ok"), pmt::from_bool(d_ctrl_ok), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_reserved"), pmt::from_uint64(d_reserved), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_seqno"), pmt::from_uint64(d_seqno), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_data_checksum"), pmt::from_uint64(d_data_checksum), d_name);

      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_syms_per_stream"), pmt::from_uint64(d_n_ofdm_syms * d_sd_num - 64), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("n_ofdm_syms"), pmt::from_uint64(d_n_ofdm_syms), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("sd_num"), pmt::from_uint64(d_sd_num), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("ctrl_syms_len"), pmt::from_uint64(ctrl_syms_len), d_name);

      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_extended"), pmt::from_uint64(d_extended), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_raw_ctrl"), pmt::make_blob(&d_raw_ctrl, sizeof(d_raw_ctrl)), d_name);

      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_nstrm"), pmt::from_uint64(d_nstrm_param), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_nstrm_phase1"), pmt::from_uint64(d_nstrm_phase1), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_nstrm_phase2"), pmt::from_uint64(d_nstrm_phase2), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_nstrm_phase3"), pmt::from_uint64(d_nstrm_phase3), d_name);

      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_current_phase"), pmt::from_uint64(d_phase), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_modtype"), pmt::from_uint64(d_modtype), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_modtype_phase1"), pmt::from_uint64(d_modtype_phase1), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_modtype_phase2"), pmt::from_uint64(d_modtype_phase2), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_modtype_phase3"), pmt::from_uint64(d_modtype_phase3), d_name);

      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_coding_rate"), pmt::from_uint64(d_code_rate), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_coding_rate_phase1"), pmt::from_uint64(d_code_rate_phase1), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_coding_rate_phase2"), pmt::from_uint64(d_code_rate_phase2), d_name);
      add_item_tag(0, d_wrt, pmt::string_to_symbol("rx_coding_rate_phase3"), pmt::from_uint64(d_code_rate_phase3), d_name);

      NCJT_LOG(d_debug, "(" << cc << ")"
                << "\n\t Added tags: " 
                << "\n\t\trx_ctrl_ok(" << d_ctrl_ok << "), rx_seqno(" << d_seqno << "), rx_data_checksum, "
                << "\n\t\trx_syms_per_stream(" << (d_n_ofdm_syms * d_sd_num - 64) << "), n_ofdm_syms(" << d_n_ofdm_syms << "), sd_num(" << d_sd_num << "), ctrl_syms_len(" << ctrl_syms_len << "), "
                << "\n\t\trx_extended(" << d_extended << "), rx_raw_ctrl, "
                << "\n\t\trx_nstrm(" << d_nstrm_param << "), rx_nstrm_phase1(" << d_nstrm_phase1 << "), rx_nstrm_phase2(" << d_nstrm_phase2 << "), rx_nstrm_phase3(" << d_nstrm_phase3 << "), "
                << "\n\t\trx_current_phase(" << d_phase << "), rx_modtype(" << d_modtype << "), rx_modtype_phase1(" << d_modtype_phase1 << "), rx_modtype_phase2(" << d_modtype_phase2 << "), rx_modtype_phase3(" << d_modtype_phase3 << "), "
                << "\n\t\trx_coding_rate(" << d_code_rate << "), rx_coding_rate_phase1(" << d_code_rate_phase1 << "), rx_coding_rate_phase2(" << d_code_rate_phase2 << "), rx_coding_rate_phase3(" << d_code_rate_phase3 << ")");
      int num_rbs = (int)std::floor(d_sd_num / RB_SIZE[d_rgmode]);
      if (d_phase == 2)
      {
        NCJT_LOG(d_debug, "(" << cc << ") Processing SNRs tag (snr_sc_linear) since tag_snr is set");
        std::ostringstream oss;

        std::vector<gr::tag_t> tags;
        get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("snr_sc_linear"));
        if (!tags.empty())
        {
          size_t blob_len = pmt::blob_length(tags[0].value);
          const float *snr_values = reinterpret_cast<const float *>(pmt::blob_data(tags[0].value));
          size_t num_elements = blob_len / sizeof(float);
          if (num_elements != static_cast<size_t>(d_sd_num))
          {
            throw std::runtime_error("[rg_demapper] ERROR: SNR tag length mismatch!");
          }
          std::vector<float> snr_rbs(num_rbs);
          for (int rb = 0; rb < num_rbs; rb++)
          {
            int start = rb * RB_SIZE[d_rgmode];
            int end = (rb != num_rbs - 1) ? start + RB_SIZE[d_rgmode] : d_sd_num;
            float snr_sum = 0.0f;
            for (int j = start; j < end; j++)
            {
              snr_sum += snr_values[j];
            }
            float snr_avg = snr_sum / (end - start);
            float snr_avg_db = 10 * std::log10(snr_avg);
            snr_rbs[rb] = snr_avg_db;
            if (d_debug) {
              oss << "\tRB_SNR[" << rb << "] = " << snr_avg_db << " dB (";
              for (int j = start; j < end; j++)
              {
                oss << snr_values[j];
                if (j < end - 1)
                  oss << ", ";
              }
              oss << ")\n";
            }
          }
          add_item_tag(0, d_wrt, pmt::string_to_symbol("snr_rbs_db"), pmt::init_f32vector(snr_rbs.size(), snr_rbs.data()), d_name);
          NCJT_LOG(d_debug, "(" << cc << ")\n" << oss.str());
        }
        else
        {
          throw std::runtime_error("[rg_demapper] ERROR: No snr_sc_linear tag found for phase 2!");
        }
      }
      else if (d_phase == 3)
      {
        NCJT_LOG(d_debug, "(" << cc << ") Processing SNRs from CTRL since tag_snr is not set");

        // int snr_bits = RB_B0[d_rgmode] + RB_Bd[d_rgmode] * (num_rbs - 1);
        // float recovered[num_rbs];
        // Get snr_bits LSB bits from the extended field
        // uint64_t packed_bits = d_extended & ((1ULL << snr_bits) - 1);


        int snr_bits = RB_B0[d_rgmode] + RB_Bd[d_rgmode] * (num_rbs - 1);
        float recovered[num_rbs];

        uint64_t mask =
            (snr_bits >= 64) ? std::numeric_limits<uint64_t>::max()
                             : ((1ULL << snr_bits) - 1ULL);

        uint64_t packed_bits = d_extended & mask;

        NCJT_LOG(d_debug, "(" << cc << ")] EXTENDED: " << std::hex << d_extended << std::dec);
        // De-quantisation
        dequantize_snrs(packed_bits, num_rbs, 
                        RB_B0[d_rgmode], RB_Bd[d_rgmode],
                        MIN_ABS, MAX_ABS,
                        MIN_DIFF, MAX_DIFF,
                        recovered);

        if (d_debug)
        {
          std::cout << "Recovered SNRs from CTRL:";
          for (auto v : recovered)
            std::cout << ' ' << v;
          std::cout << std::endl;
        }
        add_item_tag(0, d_wrt, pmt::string_to_symbol("snr_rbs_db"), pmt::init_f32vector(num_rbs, recovered), d_name);
      } else {
        NCJT_LOG(d_debug, "(" << cc << ") No SNRs processing since phase is not 2 or 3");
      }

      // Put interleaved symbols and CSI into output_items
      std::vector<const gr_complex *> data_in(d_nstrm_param);
      std::vector<const gr_complex *> csi_in(d_nstrm_param, nullptr);
      for (int s = 0; s < d_nstrm_param; s++)
      {
        data_in[s] = static_cast<const gr_complex *>(input_items[s]);
        if (d_usecsi)
        {
          csi_in[s] = static_cast<const gr_complex *>(input_items[d_nstrm_param + s]);
        }
      }

      int available_syms = min_in - ctrl_syms_len;
      if (available_syms <= 0)
      {
        NCJT_LOG(d_debug, "(" << cc << ") No data, return 0");
        return 0;
      }

      int total_syms = available_syms * d_nstrm_param;
      int out_syms = std::min(noutput_items, total_syms);
      auto out_data = static_cast<gr_complex *>(output_items[0]);
      auto out_csi = static_cast<gr_complex *>(output_items[1]);

      int idx = 0;
      for (int i = 0; i < available_syms; i++)
      {
        for (int s = 0; s < d_nstrm_param; s++)
        {
          if (idx >= out_syms)
            break;
          out_data[idx] = data_in[s][ctrl_syms_len + i];
          out_csi[idx] = d_usecsi ? csi_in[s][ctrl_syms_len + i] : gr_complex(1.0f, 0.0f);
          idx++;
        }
        if (idx >= out_syms)
          break;
      }
      // Add packet_len tag on output ports
      for (int p = 0; p < 1; p++)
      {
        add_item_tag(p, nitems_written(p), pmt::string_to_symbol("packet_len"), pmt::from_long(out_syms), d_name);
      }

      NCJT_LOG(d_debug, "(" << cc << ") Output symbols: " << out_syms);

      return out_syms;
    }

  } // namespace ncjt
} // namespace gr
