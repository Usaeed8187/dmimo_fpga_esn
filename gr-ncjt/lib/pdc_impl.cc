/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "pdc_impl.h"
#include <gnuradio/io_signature.h>
#include <chrono> // for timestamps
#include <iomanip>
#include "qam_constellation.h"
#include <random>
#include "srsran/srsvec/bit.h"
#include "common.h"
#include <gnuradio/ncjt/rg_modes.h>

namespace gr
{
  namespace ncjt
  {

    pdc::sptr pdc::make(int rgmode,
                        bool majority_enabled,
                        int num_copies,
                        int expire_ms,
                        int num_threads,
                        bool deterministic_input,
                        bool debug)
    {
      return gnuradio::make_block_sptr<pdc_impl>(rgmode, majority_enabled, num_copies, expire_ms, num_threads, deterministic_input, debug);
    }

    /*
     * The private constructor
     */
    pdc_impl::pdc_impl(int rgmode,
                       bool majority_enabled,
                       int num_copies,
                       int expire_ms,
                       int num_threads,
                       bool deterministic_input,
                       bool debug)
        : gr::block("pdc",
                    gr::io_signature::make(num_copies * 2, num_copies * 2, sizeof(gr_complex)),
                    gr::io_signature::make(1, 2, sizeof(uint8_t))),
          d_rgmode(rgmode),
          d_num_copies(num_copies),
          d_num_threads(num_threads),
          d_hll_enabled(true),
          d_majority_enabled(majority_enabled),
          d_expire_ms(expire_ms),
          d_deterministic_input(deterministic_input),
          d_debug(debug),
          d_no_inp_cnt(0)
    {
      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else
      {
        d_num_packet_syms = (RG_NUM_OFDM_SYM[rgmode] * RG_NUM_DATA_SC[rgmode] - 64);
        set_min_noutput_items(d_num_packet_syms * 8);
      }

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

      auto decoder_factory = srsran::create_ldpc_decoder_factory_sw("avx2");
      auto dematcher_factory = srsran::create_ldpc_rate_dematcher_factory_sw("avx2");

      if (decoder_factory && dematcher_factory)
      {
        d_ldpc_decoder = decoder_factory->create();
        d_ldpc_dematcher = dematcher_factory->create();
      }
      else
      {
        throw std::runtime_error(
            "[PDC] ERROR: LDPC factories could not be created.");
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

      set_tag_propagation_policy(TPP_DONT);
    }

    pdc_impl::~pdc_impl()
    {
    }

    void pdc_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
      for (size_t i = 0; i < ninput_items_required.size(); i++)
      {
        ninput_items_required[i] = 0;
      }
    }

    int pdc_impl::general_work(int noutput_items, gr_vector_int &ninput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
    {
      std::vector<const uint8_t *> inps;
      int max_in = 0;
      for (size_t i = 0; i < ninput_items.size(); i++)
      {
        inps.push_back(static_cast<const uint8_t *>(input_items[i]));
        if (ninput_items[i] > max_in)
        {
          max_in = ninput_items[i];
        }
      }
      if (d_pendings.empty())
      {
        if (max_in < 1 && d_no_inp_cnt < 100)
        {
          d_no_inp_cnt++;
          // Sleep for 0.001 seconds
          usleep(100);
          produce(0, 0);
          if (d_majority_enabled && d_hll_enabled)
          {
            produce(1, 0);
          }
          return WORK_CALLED_PRODUCE;
        }
      }
      d_no_inp_cnt = 0;

      if (max_in && d_debug)
      {
        std::stringstream ss;
        ss << "[pdc_impl::general_work] Called, noutput_items=" << noutput_items
           << std::endl;
        for (size_t p = 0; p < static_cast<size_t>(d_num_copies); p++)
        {
          if (ninput_items[p])
          {
            ss << "\tinput port " << p << " => " << ninput_items[p] << " items";
            std::vector<gr::tag_t> tags1;
            get_tags_in_window(tags1, p, 0, ninput_items[p], pmt::string_to_symbol("rx_seqno"));
            if (tags1.empty())
            {
              ss << " (no rx_seqno tag)" << std::endl;
            }
            else
            {
              for (const auto &tag : tags1)
              {
                ss << " (rx_seqno=" << pmt::to_uint64(tag.value) << " at " << tag.offset << ")";
              }
              ss << std::endl;
            }
          }
        }
        std::cout << ss.str();
      }

      std::vector<gr::tag_t> tags1;
      std::vector<gr::tag_t> tags2;
      bool tag_ctrl_ok = false;
      int64_t _seqno = 0;
      uint64_t _packet_len_syms = 0;
      int64_t _modtype = 0;
      int64_t _p2modtype = 0;
      int64_t _p3modtype = 0;
      int64_t _rx_coding_rate;
      uint64_t _rx_data_checksum = 0;
      uint64_t _ctrl_syms_len = 0;
      uint64_t _sd_num = 0;
      uint64_t _tt = 0;

      for (size_t qam_inp = 0; qam_inp < static_cast<size_t>(d_num_copies); qam_inp++)
      {
        // std::cout << "Nima: Processing input port " << qam_inp
        //           << ", ninput_items=" << ninput_items[qam_inp]
        //           << ", ninput_items[qam_inp + d_num_copies]=" << ninput_items[qam_inp + d_num_copies] 
        //           << ", d_num_packet_syms=" << d_num_packet_syms << std::endl;
        if (ninput_items[qam_inp] >= d_num_packet_syms && ninput_items[qam_inp + d_num_copies] >= d_num_packet_syms)
        {
          // std::cout << "\tNima: Processing input port " << qam_inp << std::endl;
          // tag: packet_len
          get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("packet_len"));
          _packet_len_syms = pmt::to_uint64(tags2[0].value);
          if (_packet_len_syms != d_num_packet_syms)
          {
            throw std::runtime_error("[pdc_impl] ERROR: Only one stream is supported for now!");
          }
          // tag: rx_ctrl_ok
          get_tags_in_window(tags1, qam_inp, 0, 1, pmt::string_to_symbol("rx_ctrl_ok"));
          if (tags1.size() == 0)
          {
            throw std::runtime_error("[pdc_impl] ERROR: No rx_ctrl_ok tag found in input stream");
          }
          tag_ctrl_ok = pmt::to_bool(tags1[0].value);
          if (tag_ctrl_ok)
          {
            _tt += 1;
            // tag: rx_modtype
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_modtype"));
            _modtype = pmt::to_uint64(tags2[0].value);
            // tag: rx_modtype_phase2
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_modtype_phase2"));
            _p2modtype = pmt::to_uint64(tags2[0].value);
            // tag: rx_modtype_phase3
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_modtype_phase3"));
            _p3modtype = pmt::to_uint64(tags2[0].value);
            // tag: rx_coding_rate
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_coding_rate"));
            _rx_coding_rate = pmt::to_uint64(tags2[0].value);
            // tag: rx_data_checksum
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_data_checksum"));
            _rx_data_checksum = pmt::to_uint64(tags2[0].value);
            // tag: snr_rbs_db

            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("snr_rbs_db"));

            if (!pmt::is_f32vector(tags2[0].value))
            {
              throw std::runtime_error("[pdc_impl] ERROR: snr_rbs_db tag is not a f32vector!");
            }

            size_t num_snr_elements;
            auto snr_tag_val = pmt::f32vector_elements(tags2[0].value, num_snr_elements);

            float *_snr_rbs_db = (float *)malloc(num_snr_elements * sizeof(float));
            // print SNR values
            // std::cout << "SNR values: ";
            for (size_t i = 0; i < num_snr_elements; i++)
            {
              _snr_rbs_db[i] = snr_tag_val[i];
              // std::cout << _snr_rbs_db[i] << " ";
            }
            // std::cout << std::endl;
            std::memcpy(_snr_rbs_db, snr_tag_val, num_snr_elements * sizeof(float));

            // tag: ctrl_syms_len
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("ctrl_syms_len"));
            _ctrl_syms_len = pmt::to_uint64(tags2[0].value);
            // tag: sd_num
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("sd_num"));
            _sd_num = pmt::to_uint64(tags2[0].value);
            // tag: rx_seqno
            get_tags_in_window(tags2, qam_inp, 0, 1, pmt::string_to_symbol("rx_seqno"));
            _seqno = (int64_t)pmt::to_uint64(tags2[0].value);
            // Get QAM and CSI
            gr_complex *_buf = (gr_complex *)malloc(_packet_len_syms * sizeof(gr_complex));
            std::memcpy(_buf, inps[qam_inp], _packet_len_syms * sizeof(gr_complex));
            gr_complex *_csi_buf = (gr_complex *)malloc(_packet_len_syms * sizeof(gr_complex));
            std::memcpy(_csi_buf, inps[qam_inp + d_num_copies], _packet_len_syms * sizeof(gr_complex));
            // Compute current timestamp in nanoseconds
            auto now = std::chrono::system_clock::now();
            uint64_t ts = _tt + std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            bool found = false;
            for (auto &pending : d_pendings)
            {
              auto it = std::find_if(pending.begin(), pending.end(),
                                     [_seqno](const auto &pair)
                                     {
                                       return pair.second.seqno == _seqno;
                                     });
              if (it != pending.end())
              {
                // Found the seqno!
                pending[qam_inp] = {_buf,
                                    _csi_buf,
                                    _seqno,
                                    _packet_len_syms,
                                    _modtype,
                                    _p2modtype,
                                    _p3modtype,
                                    _rx_coding_rate,
                                    _rx_data_checksum,
                                    _snr_rbs_db,
                                    _ctrl_syms_len,
                                    _sd_num,
                                    ts};
                // std::cout << "\t\tUpdated existing entry in d_pendings for [qam_inp] " << qam_inp
                //           << ", seqno=" << _seqno << std::endl;
                found = true;
                break;
              }
            }
            if (!found)
            {
              // Not found, create a new entry
              bool add_flag_last = false;
              if (d_pendings.empty())
              {
                add_flag_last = true;
              }
              else
              {
                // Get seqno of any item in the last element of the deque
                auto last_seqno = d_pendings.back().begin()->second.seqno;
                if (_seqno > last_seqno)
                {
                  add_flag_last = true;
                }
              }
              if (add_flag_last)
              {
                std::unordered_map<int, packet> new_entry;
                new_entry[qam_inp] = {_buf,
                                      _csi_buf,
                                      _seqno,
                                      _packet_len_syms,
                                      _modtype,
                                      _p2modtype,
                                      _p3modtype,
                                      _rx_coding_rate,
                                      _rx_data_checksum,
                                      _snr_rbs_db,
                                      _ctrl_syms_len,
                                      _sd_num,
                                      ts};
                d_pendings.push_back(new_entry);
                // std::cout << "\t\tAdded new entry to d_pendings for [qam_inp] " << qam_inp
                //           << ", seqno=" << _seqno << std::endl;
              }
              else
              {
                auto first_seqno = d_pendings.front().begin()->second.seqno;
                auto last_seqno = d_pendings.back().begin()->second.seqno;
                if ((_seqno < last_seqno) && (_seqno > first_seqno))
                {
                  // Insert in the middle
                  std::unordered_map<int, packet> new_entry;
                  new_entry[qam_inp] = {_buf,
                                        _csi_buf,
                                        _seqno,
                                        _packet_len_syms,
                                        _modtype,
                                        _p2modtype,
                                        _p3modtype,
                                        _rx_coding_rate,
                                        _rx_data_checksum,
                                        _snr_rbs_db,
                                        _ctrl_syms_len,
                                        _sd_num,
                                        ts};
                  for (auto it = d_pendings.begin(); it != d_pendings.end(); ++it)
                  {
                    if (_seqno < it->begin()->second.seqno)
                    {
                      d_pendings.insert(it, new_entry);
                      break;
                    }
                  }
                }
                else
                {
                  if (d_debug)
                  {
                    std::cout << "\t\tSkipping adding new entry to d_pendings for [qam_inp] " << qam_inp
                              << ", seqno=" << _seqno << ", first_seqno=" << first_seqno << ", last_seqno=" << last_seqno << std::endl;
                  }
                  free(_buf);
                  free(_csi_buf);
                  free(_snr_rbs_db);
                }
              }
            }
          }
          else
          {
            // DEBUG
            std::cout << " [pdc_impl] ctrl_ok=false, skipping this packet" << std::endl;
          }
          consume(qam_inp, _packet_len_syms);
          consume(qam_inp + d_num_copies, _packet_len_syms);
          if (d_debug)
          {
            std::cout << "Consumed " << _packet_len_syms << " symbols from input port " << qam_inp
                      << " (amd input port " << qam_inp + d_num_copies << ")" << std::endl;
          }
        }
      }

      // Look for a fully-assembled or expired packet
      int process_till = -1;
      for (size_t j = 0; j < d_pendings.size(); j++)
      {
        if (d_pendings[j].size() == (size_t)(d_num_copies))
        {
          process_till = j;
          if (d_debug)
          {

            std::cout << "\t Found complete packet seqno: " << d_pendings[j].begin()->second.seqno
                      << " in d_pendings[" << j << "] with " << d_pendings[j].size() << " copies."
                      << std::endl;
          }
          break;
        }
        auto now = std::chrono::system_clock::now();
        uint64_t current_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        auto it = std::find_if(d_pendings[j].begin(),
                               d_pendings[j].end(),
                               [current_time, this](const auto &pair)
                               {
                                 return (current_time - pair.second.timestamp) > (uint64_t(d_expire_ms) * 1000000ULL);
                               });
        if (it != d_pendings[j].end())
        {
          process_till = j;
          if (d_debug)
          {
            std::cout << "\t Found expired packet seqno: " << it->second.seqno
                      << " in d_pendings[" << j << "] with " << d_pendings[j].size() << " copies."
                      << std::endl;
          }
          break;
        }
      }

      // if (d_debug)
      // {
      //   std::cout << "process_till: " << process_till << std::endl;
      // }

      for (int q = 0; q <= process_till; q++)
      {
        auto pending = std::move(d_pendings.front());
        d_pendings.pop_front();
        auto seqno = pending.begin()->second.seqno;
        int packet_len = pending.begin()->second.packet_len;
        // std::cout << "Nima: packet_len = " << packet_len << std::endl;
        // auto modtype = pending.begin()->second.modtype;
        // auto modtype = pending.begin()->second.p3modtype;
        auto p2modtype = pending.begin()->second.p2modtype;
        auto p3modtype = pending.begin()->second.p3modtype;
        auto rx_coding_rate = pending.begin()->second.rx_coding_rate;
        auto rx_data_checksum = pending.begin()->second.rx_data_checksum;
        auto sd_num = pending.begin()->second.sd_num;

        int copies_count = (int)pending.size();

        if (d_debug)
        {
          NCJT_LOG(d_debug, "Processing packet with seqno: " << seqno
                    << ", packet_len: " << packet_len
                    << ", copies_count: " << copies_count
                    << ", p2modtype: " << p2modtype
                    << ", p3modtype: " << p3modtype
                    << ", rx_coding_rate: " << rx_coding_rate
                    << ", rx_data_checksum: " << rx_data_checksum);
        }
        int max_modtype = 0;
        for (const auto &pair : pending)
        {
          if (pair.second.modtype > max_modtype)
          {
            max_modtype = pair.second.modtype;
          }
          NCJT_LOG(d_debug, "\tCopy " << pair.first << " modtype: " << pair.second.modtype);
        }

        int max_bits_per_pkt = packet_len * max_modtype;
        int p2max_bits_per_pkt = packet_len * p2modtype;

        NCJT_LOG(d_debug, "max_modtype: " << max_modtype
                  << ", max_bits_per_pkt: " << max_bits_per_pkt
                  << ", packet_len: " << packet_len);

        /**************************************************************
         * 1) MAJORITY VOTING decode
         **************************************************************/
        // We'll demap each copy's QAM symbol -> bits[0..modtype-1],
        // and accumulate them. Then we threshold them at 'copies/2'.
        std::vector<float> llrs_majority(max_bits_per_pkt, 0);
        std::vector<uint8_t> bits_majority(max_bits_per_pkt, 0);

        if (d_majority_enabled)
        {
          // Accumulate
          for (auto it = pending.begin(); it != pending.end(); ++it)
          {
            auto &pkt = it->second; // struct packet
            for (int sym_idx = 0; sym_idx < packet_len; sym_idx++)
            {
              float x = pkt.buf[sym_idx].real();
              float y = pkt.buf[sym_idx].imag();
              float csi_val = pkt.csi_buf[sym_idx].real();
              uint8_t tmp_bits[8];
              gr::ncjt::demap_symbol(x, y, csi_val, it->second.modtype, tmp_bits);

              // We assume single stream => just place them contiguously
              int out_sym_offset = sym_idx * it->second.modtype;
              for (int b = 0; b < it->second.modtype; b++)
              {
                llrs_majority[out_sym_offset + b] += tmp_bits[b];
              }
            }
          }
          for (int i = 0; i < p2max_bits_per_pkt; i++)
          {
            llrs_majority[i] /= copies_count; // Normalize by number of copies
            llrs_majority[i] -= 0.5f; // Center around zero
            // Now threshold
            bits_majority[i] = (llrs_majority[i] > 0) ? 1 : 0;
          }
        }

        /**************************************************************
         * 2) HARD-LOG-LIKELIHOOD VANILLA decode
         **************************************************************/
        // We'll feed all copies into the HardLogLikelihoodVanilla object:
        std::vector<float> rx_real(packet_len * copies_count);
        std::vector<float> rx_imag(packet_len * copies_count);
        std::vector<float> SNRs(packet_len * copies_count);
        std::vector<float> llrs_hll;
        std::vector<uint8_t> bits_hll(p2max_bits_per_pkt);

        pmt::pmt_t rb_snrs_dict = pmt::make_dict();
        if (d_hll_enabled)
        {
          // Fill those arrays
          int copy_cnt = 0;
          for (auto it = pending.begin(); it != pending.end(); ++it)
          {
            int copy_id = it->first;
            auto &pkt = it->second;
            //
            std::vector<float> snr_sc_linear(sd_num, -10.0f);
            int num_rbs = (int)std::floor(sd_num / RB_SIZE[d_rgmode]);
            rb_snrs_dict = pmt::dict_add(rb_snrs_dict,
                                         pmt::from_long(copy_id),
                                         pmt::init_f32vector(num_rbs, pkt.snr_rbs_db));
            // First extract SNR values for each RB
            for (int rb = 0; rb < num_rbs; rb++)
            {
              int start = rb * RB_SIZE[d_rgmode];
              int end = (rb != num_rbs - 1) ? start + RB_SIZE[d_rgmode] : sd_num;
              // std::cout << "RB[" << rb << "] SNR (dB) = " << pkt.snr_rbs_db[rb] << std::endl;
              float snr_linear = pow(10.0f, pkt.snr_rbs_db[rb] / 10.0f);
              // std::cout << "RB[" << rb << "] SNR (linear) = " << snr_linear << std::endl;
              for (int j = start; j < end; j++)
              {
                snr_sc_linear[j] = snr_linear;
                // DEBUG
                // std::cout << "\tPDC_SNR[" << j << "] = " << snr_sc_linear[j] << std::endl;
              }
            }

            // Account for the CTRL syms offset
            int sc_ind = pkt.ctrl_syms_len;

            //////////////////
            // std::cout << "*** NIMA: copy_cnt = " << copy_cnt
            //           << ", packet_len = " << packet_len
            //           << ", tmp_remap_buf size = " << packet_len * it->second.modtype
            //           << std::endl;
            
            // Make a temporary buffer for remapping
            uint8_t *tmp_remap_buf = (uint8_t *)malloc(packet_len * sizeof(uint8_t) * it->second.modtype);
            for (int sym_idx = 0; sym_idx < packet_len; sym_idx++)
            {
              int idx = sym_idx * copies_count + copy_cnt;
              SNRs[idx] = snr_sc_linear[sc_ind % sd_num];
              sc_ind++;
              //
              float x = pkt.buf[sym_idx].real();
              float y = pkt.buf[sym_idx].imag();
              float csi_val = pkt.csi_buf[sym_idx].real();
              uint8_t tmp_bits[8]; // up to 8 bits
              gr::ncjt::demap_symbol(x, y, csi_val, it->second.modtype, tmp_bits);
              int out_sym_offset = sym_idx * it->second.modtype;
              for (int b = 0; b < it->second.modtype; b++)
              {
                tmp_remap_buf[out_sym_offset + b] = tmp_bits[b];
              }
            }
            // Remap
            for (int sym_idx = 0; sym_idx < packet_len; sym_idx++) {
              int idx = sym_idx * copies_count + copy_cnt;
            // for (int k = 0; k < packet_len; k++) {
              int period_offset = sym_idx * it->second.p2modtype;
              int val = 0;
              for (int b = 0; b < it->second.p2modtype; b++)
              {
                int bit_index = period_offset + b;
                val |= (tmp_remap_buf[bit_index] << b);
              }
              gr_complex sym;
              switch (it->second.p2modtype)
              {
              case 2:
                sym = CONST_QPSK[val];
                break;
              case 4:
                sym = CONST_16QAM[val];
                break;
              case 6:
                sym = CONST_64QAM[val];
                break;
              case 8:
                sym = CONST_256QAM[val];
                break;
              default:
                throw std::runtime_error("Unsupported modulation type for remapping");
              }
              rx_real[idx] = sym.real();
              rx_imag[idx] = sym.imag();
            }
            free(tmp_remap_buf);
            //////////////////
            copy_cnt++;
          }

          // Run HardLogLikelihoodVanilla -> returns bit LLRs if sum_over_rx=true + return_bit_llrs=true

          d_hlls[p2modtype][copies_count]->Compute(rx_real.data(),
                                                 rx_imag.data(),
                                                 SNRs.data(),
                                                 packet_len,
                                                 copies_count,
                                                 llrs_hll,
                                                 d_num_threads);
          // Reverse the order of the LLRs
          for (size_t d = 0; d < llrs_hll.size(); d += p2modtype)
          {
            std::reverse(llrs_hll.begin() + d, llrs_hll.begin() + d + p2modtype);
          }

          // Now we "hard"-decide bits from LLR sign:
          for (int i = 0; i < p2max_bits_per_pkt; i++)
          {
            bits_hll[i] = (llrs_hll[i] >= 0.0f) ? 1 : 0;
          }
        }

        /**************************************************************
         * 3) VERIFICATION / LOGGING
         **************************************************************/
        if (d_debug && d_hll_enabled && d_majority_enabled)
        {
          std::cout << "The first 5 symbols:" << std::endl;
          std::cout << "Number of copies: " << copies_count << std::endl;

          for (int p = 0; p < 5; p++)
          {
            // Majority bits
            std::cout << "[";
            for (int b = 0; b < p2modtype; b++)
            {
              std::cout << (int)bits_majority[p * p2modtype + b];
            }
            std::cout << "]\t";
            // Majority LLRs
            std::cout << "[";
            for (int b = 0; b < p2modtype; b++)
            {
              std::cout << std::setw(5) << std::fixed << std::setprecision(2) << llrs_majority[p * p2modtype + b];
              if (b < p2modtype - 1)
                std::cout << ", ";
            }
            std::cout << "]\t";
            // HLL bits
            std::cout << "[";
            for (int b = 0; b < p2modtype; b++)
            {
              std::cout << (int)bits_hll[p * p2modtype + b];
            }
            std::cout << "]\t";
            // HLL LLRs
            std::cout << "[";
            for (int b = 0; b < p2modtype; b++)
            {
              std::cout << std::setw(5) << std::fixed << std::setprecision(2) << llrs_hll[p * p2modtype + b];
              if (b < p2modtype - 1)
                std::cout << ", ";
            }
            std::cout << "]\t";
            // Symbols (copies)
            for (auto it = pending.begin(); it != pending.end(); ++it)
            {
              auto &pkt = it->second;
              std::cout << "(" << std::setw(6) << std::fixed << std::setprecision(2) << pkt.buf[p].real()
                        << " + "
                        << std::setw(6) << std::fixed << std::setprecision(2) << pkt.buf[p].imag()
                        << "i)\t";
              if (std::next(it) != pending.end())
              {
                std::cout << ", ";
              }
            }
            std::cout << std::endl;
          }
        }

        /**************************************************************
         * 4) LDPC decode
         **************************************************************/
        uint8_t *majority_buf = nullptr;
        int majority_buf_len = 0;
        uint8_t *hll_buf = nullptr;
        int hll_buf_len = 0;
        if (rx_coding_rate > 0)
        {
          double R = code_rates[rx_coding_rate];
          int message_len_bits = int(std::floor(p2max_bits_per_pkt * R));
          message_len_bits = message_len_bits + (p2modtype - message_len_bits % p2modtype);
          int seg_out_size = packet_len;
          int num_segs = (packet_len * p2modtype) / seg_out_size;
          int base_in_bits_per_seg = message_len_bits / num_segs;
          int remainder = message_len_bits % num_segs;
          ///
          if (d_majority_enabled)
          {
            std::vector<srsran::log_likelihood_ratio> srs_llrs_majority(packet_len * p2modtype);
            for (int k = 0; k < packet_len * p2modtype; k++)
            {
              srs_llrs_majority[k] = (int)(llrs_majority[k] * -20); // bits_majority[k] ? -10 : 10;
              if (d_debug && k < 5)
                std::cout << "llrs_majority[" << k << "] = " << ((int)srs_llrs_majority[k]) << std::endl;
            }
            // LDPC decode for MAJORITY
            majority_buf = (uint8_t *)malloc(message_len_bits);
            int start_idx = 0;
            for (int seg = 0; seg < num_segs; seg++)
            {
              int in_bits_per_seg = base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
              auto majority_bits = gr::ncjt::ldpc_decode(in_bits_per_seg,
                                                         std::vector<srsran::log_likelihood_ratio>(srs_llrs_majority.begin() + start_idx,
                                                                                                   srs_llrs_majority.begin() + start_idx + seg_out_size),
                                                         d_ldpc_decoder.get(),
                                                         d_ldpc_dematcher.get());
              std::memcpy(majority_buf + majority_buf_len, majority_bits.data(), majority_bits.size());
              majority_buf_len += majority_bits.size();
              start_idx += seg_out_size;
            }
          }
          if (d_hll_enabled)
          {
            std::vector<srsran::log_likelihood_ratio> srs_llrs_hll(packet_len * p2modtype);
            for (int k = 0; k < packet_len * p2modtype; k++)
            {
              {
                float temp = llrs_hll[k] * -10;
                if (temp > 30)
                  temp = 30;
                else if (temp < -30)
                  temp = -30;
                srs_llrs_hll[k] = static_cast<int>(temp);
              }

              // srs_llrs_hll[k] = (int)llrs_hll[k] * 10;
              if (d_debug && k < 5)
                std::cout << "llrs_hll[" << k << "] = " << ((int)srs_llrs_hll[k]) << std::endl;
            }
            // LDPC decode for HLL
            hll_buf = (uint8_t *)malloc(message_len_bits);
            int start_idx = 0;
            for (int seg = 0; seg < num_segs; seg++)
            {
              int in_bits_per_seg = base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
              auto hll_bits = gr::ncjt::ldpc_decode(in_bits_per_seg,
                                                    std::vector<srsran::log_likelihood_ratio>(srs_llrs_hll.begin() + start_idx,
                                                                                              srs_llrs_hll.begin() + start_idx + seg_out_size),
                                                    d_ldpc_decoder.get(),
                                                    d_ldpc_dematcher.get());
              std::memcpy(hll_buf + hll_buf_len, hll_bits.data(), hll_bits.size());
              hll_buf_len += hll_bits.size();
              start_idx += seg_out_size;
            }
          }
        }
        else
        {
          // No coding, just copy the bits
          if (d_majority_enabled)
          {
            majority_buf = (uint8_t *)malloc(p2max_bits_per_pkt);
            std::memcpy(majority_buf, bits_majority.data(), p2max_bits_per_pkt);
            majority_buf_len = p2max_bits_per_pkt;
          }
          if (d_hll_enabled)
          {
            hll_buf = (uint8_t *)malloc(p2max_bits_per_pkt);
            std::memcpy(hll_buf, bits_hll.data(), p2max_bits_per_pkt);
            hll_buf_len = p2max_bits_per_pkt;
          }
        }
        /**************************************************************
         * Determine ports
         **************************************************************/
        int hll_port = 0;
        int majority_port = 1;
        /**************************************************************
         * 5) If deterministic input => measure coded BER and uncoded BER
         **************************************************************/
        if (d_deterministic_input)
        {
          // Recalculate how many *info bits* the mapper used
          int frame_data_syms = packet_len;
          int frame_data_bits_out = frame_data_syms * p2modtype;
          double R = code_rates[rx_coding_rate];
          int in_bits_needed = frame_data_bits_out;
          if (rx_coding_rate > 0)
          {
            in_bits_needed = int(std::floor(frame_data_bits_out * R));
            int rem = in_bits_needed % p2modtype;
            if (rem != 0)
            {
              in_bits_needed += (p2modtype - rem);
            }
          }
          // Generate the same PRNG bits used by mapper_muxer_impl
          std::mt19937 gen(seqno);
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
          double coded_ber_majority = 0.0;
          if (d_majority_enabled)
          {
            int compare_len = std::min(majority_buf_len, in_bits_needed);
            int bit_errors = 0;
            for (int i = 0; i < compare_len; i++)
            {
              if (majority_buf[i] != reference_bits[i])
              {
                bit_errors++;
              }
            }
            coded_ber_majority = (compare_len > 0)
                                     ? double(bit_errors) / double(compare_len)
                                     : 0.0;
          }
          // ---------------------------
          double coded_ber_hll = 0.0;
          if (d_hll_enabled)
          {
            int compare_len = std::min(hll_buf_len, in_bits_needed);
            int bit_errors = 0;
            for (int i = 0; i < compare_len; i++)
            {
              if (hll_buf[i] != reference_bits[i])
              {
                bit_errors++;
              }
            }
            coded_ber_hll = (compare_len > 0)
                                ? double(bit_errors) / double(compare_len)
                                : 0.0;
          }
          // ---------------------------
          // 5b) Uncoded BER
          //     Re-encode the reference_bits so we get the same coded bits
          //     that the TX used, then compare to d_coded_buf
          // ---------------------------
          std::vector<uint8_t> reference_coded_bits;
          if (rx_coding_rate > 0)
          {
            int seg_out_size = packet_len;
            int num_segs = (packet_len * p2modtype) / seg_out_size;
            int base_in_bits_per_seg = in_bits_needed / num_segs;
            int remainder = in_bits_needed % num_segs;
            if (d_majority_enabled)
            {
              reference_coded_bits.reserve(majority_buf_len);
            }
            else if (d_hll_enabled)
            {
              reference_coded_bits.reserve(hll_buf_len);
            }
            int start_idx = 0;
            for (int seg = 0; seg < num_segs; seg++)
            {
              int in_bits_per_seg = base_in_bits_per_seg + (seg == num_segs - 1 ? remainder : 0);
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
          }
          else
          {
            // no coding => coded bits = raw bits
            reference_coded_bits = reference_bits;
          }
          double uncoded_ber_majority = 0.0;
          if (d_majority_enabled)
          {
            int uncoded_compare_len =
                std::min((int)reference_coded_bits.size(), (int)bits_majority.size());
            int uncoded_errors = 0;
            for (int i = 0; i < uncoded_compare_len; i++)
            {
              if (reference_coded_bits[i] != bits_majority[i])
              {
                uncoded_errors++;
              }
            }
            uncoded_ber_majority = (uncoded_compare_len > 0)
                                       ? double(uncoded_errors) / double(uncoded_compare_len)
                                       : 0.0;
          }
          double uncoded_ber_hll = 0.0;
          if (d_hll_enabled)
          {
            int uncoded_compare_len =
                std::min((int)reference_coded_bits.size(), (int)bits_hll.size());
            int uncoded_errors = 0;
            for (int i = 0; i < uncoded_compare_len; i++)
            {
              if (reference_coded_bits[i] != bits_hll[i])
              {
                uncoded_errors++;
              }
            }
            uncoded_ber_hll = (uncoded_compare_len > 0)
                                  ? double(uncoded_errors) / double(uncoded_compare_len)
                                  : 0.0;
          }
          // Add tags for coded and uncoded BER
          if (d_majority_enabled)
          {
            add_item_tag(majority_port, nitems_written(majority_port),
                         pmt::string_to_symbol("rx_coded_ber"),
                         pmt::from_double(coded_ber_majority),
                         pmt::string_to_symbol(this->name()));
            add_item_tag(majority_port, nitems_written(majority_port),
                         pmt::string_to_symbol("rx_uncoded_ber"),
                         pmt::from_double(uncoded_ber_majority),
                         pmt::string_to_symbol(this->name()));
          }
          if (d_hll_enabled)
          {
            add_item_tag(hll_port, nitems_written(hll_port),
                         pmt::string_to_symbol("rx_coded_ber"),
                         pmt::from_double(coded_ber_hll),
                         pmt::string_to_symbol(this->name()));
            add_item_tag(hll_port, nitems_written(hll_port),
                         pmt::string_to_symbol("rx_uncoded_ber"),
                         pmt::from_double(uncoded_ber_hll),
                         pmt::string_to_symbol(this->name()));
            add_item_tag(hll_port, nitems_written(hll_port),
                         pmt::string_to_symbol("hll_snr_rbs_db"),
                         rb_snrs_dict,
                         pmt::string_to_symbol(this->name()));
          }
        }
        /**************************************************************
         * 6) LDPC decode
         **************************************************************/
        bool crc_ok_majority = false;
        bool crc_ok_hll = false;
        if (d_majority_enabled)
        {
          uint16_t crc_majority = compute_crc16(majority_buf, majority_buf_len);
          crc_ok_majority = (crc_majority == (uint16_t)rx_data_checksum);
          if (d_debug)
          {
            if (crc_ok_majority)
            {
              std::cout << "MAJORITY: CRC OK!"
                        << " crc: " << std::hex << crc_majority
                        << ", rx_data_checksum: " << std::hex << rx_data_checksum
                        << std::dec << std::endl;
            }
            else
            {
              std::cout << "MAJORITY: CRC FAIL!"
                        << " crc: " << std::hex << crc_majority
                        << ", rx_data_checksum: " << std::hex << rx_data_checksum
                        << std::dec << std::endl;
            }
          }
        }
        if (d_hll_enabled)
        {
          uint16_t crc_hll = compute_crc16(hll_buf, hll_buf_len);
          crc_ok_hll = (crc_hll == (uint16_t)rx_data_checksum);
          if (d_debug)
          {
            if (crc_ok_hll)
            {
              std::cout << "HLL: CRC OK!"
                        << " crc: " << std::hex << crc_hll
                        << ", rx_data_checksum: " << std::hex << rx_data_checksum
                        << std::dec << std::endl;
            }
            else
            {
              std::cout << "HLL: CRC FAIL!"
                        << " crc: " << std::hex << crc_hll
                        << ", rx_data_checksum: " << std::hex << rx_data_checksum
                        << std::dec << std::endl;
            }
          }
        }
        // Free the buffers for each copy
        for (auto &pair : pending)
        {
          free(pair.second.buf);
          pair.second.buf = nullptr;
          free(pair.second.csi_buf);
          pair.second.csi_buf = nullptr;
          free(pair.second.snr_rbs_db);
        }
        if (d_hll_enabled)
        {
          auto out = static_cast<uint8_t *>(output_items[hll_port]);
          // std::memcpy(out, hll_buf, hll_buf_len);
          // Pack modtype bits into bytes
          for (int d = 0; d < hll_buf_len / p2modtype; d++)
          {
            uint8_t val = 0;
            for (int b = 0; b < p2modtype; b++)
            {
              val = (val << 1) | hll_buf[d * p2modtype + b];
            }
            out[d] = val;
          }

          free(hll_buf);
          hll_buf = nullptr;
          // Add CRC tag for HLL
          add_item_tag(hll_port, nitems_written(hll_port), pmt::string_to_symbol("rx_data_crc"),
                       pmt::from_bool(crc_ok_hll), pmt::string_to_symbol(this->name()));
          add_item_tag(hll_port, nitems_written(hll_port), pmt::string_to_symbol("packet_len"),
                       pmt::from_long(hll_buf_len / p2modtype),
                       pmt::string_to_symbol(this->name()));
        }
        if (d_majority_enabled)
        {
          auto out = static_cast<uint8_t *>(output_items[majority_port]);
          // std::memcpy(out, majority_buf, majority_buf_len);
          // Pack modtype bits into bytes
          for (int d = 0; d < majority_buf_len / p2modtype; d++)
          {
            uint8_t val = 0;
            for (int b = 0; b < p2modtype; b++)
            {
              val = (val << 1) | majority_buf[d * p2modtype + b];
            }
            out[d] = val;
          }

          free(majority_buf);
          majority_buf = nullptr;
          // Add CRC tag for MAJORITY
          add_item_tag(majority_port, nitems_written(majority_port), pmt::string_to_symbol("rx_data_crc"),
                       pmt::from_bool(crc_ok_majority), pmt::string_to_symbol(this->name()));
          add_item_tag(majority_port, nitems_written(majority_port), pmt::string_to_symbol("packet_len"),
                       pmt::from_long(majority_buf_len / p2modtype),
                       pmt::string_to_symbol(this->name()));
        }
        for (int pp = 0; pp < int(d_hll_enabled) + int(d_majority_enabled); pp++)
        {
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_ctrl_ok"),
                       pmt::from_bool(true),
                       pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_seqno"),
                       pmt::from_uint64(pending.begin()->second.seqno),
                       pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_modtype"),
                       pmt::from_uint64(pending.begin()->second.p2modtype), // TODO: Check this
                       pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_nstrm"),
                       pmt::from_uint64(pending.size()),
                       pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_coding_rate"),
                       pmt::from_uint64(pending.begin()->second.rx_coding_rate),
                       pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_data_checksum"),
                       pmt::from_uint64(pending.begin()->second.rx_data_checksum),
                       pmt::string_to_symbol(this->name()));
          // add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("avg_snr"),
          //              pmt::from_float(pending.begin()->second.snr_db),
          //              pmt::string_to_symbol(this->name()));
          add_item_tag(pp, nitems_written(pp), pmt::string_to_symbol("rx_num_copies"),
                       pmt::from_uint64(copies_count),
                       pmt::string_to_symbol(this->name()));
        }

        if (d_hll_enabled)
        {
          produce(hll_port, hll_buf_len / p2modtype);
        }
        if (d_majority_enabled)
        {
          produce(majority_port, majority_buf_len / p2modtype);
        }
        return WORK_CALLED_PRODUCE;
      }
      produce(0, 0);
      if (d_majority_enabled && d_hll_enabled)
      {
        produce(1, 0);
      }
      return WORK_CALLED_PRODUCE;
    }
  } /* namespace ncjt */
} /* namespace gr */
