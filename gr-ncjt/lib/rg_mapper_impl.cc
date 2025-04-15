/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "rg_mapper_impl.h"
#include "utils.h"
#include <gnuradio/io_signature.h>
#include <algorithm> // for std::min, std::max
#include <iostream>  // for debug prints
#include <fstream>   // for file I/O
#include <cstdio>    // for FILE*, fopen, fread, etc.
#include <cstring>   // for memcpy
#include <cmath>     // for M_PI
#include <stdexcept>

// #define VERSION 2

namespace gr::ncjt
{

  rg_mapper::sptr rg_mapper::make(int nstrm,
                                  int n_ofdm_syms,
                                  int sc_num,
                                  const std::vector<int> &pilot_sc_ind,
                                  bool addcs,
                                  bool debug,
                                  int numue,
                                  int ueidx,
                                  bool mucpt,
                                  bool addltf,
                                  const char *ltfdata)
  {
    return gnuradio::make_block_sptr<rg_mapper_impl>(
        nstrm, n_ofdm_syms, sc_num, pilot_sc_ind, addcs, debug, numue, ueidx, mucpt, addltf, ltfdata);
  }

  rg_mapper_impl::rg_mapper_impl(int nstrm,
                                       int n_ofdm_syms,
                                       int sc_num,
                                       const std::vector<int> &pilot_sc_ind,
                                       bool addcs,
                                       bool debug,
                                       int numue,
                                       int ueidx,
                                       bool mucpt,
                                       bool addltf,
                                       const char *ltfdata)
      : gr::tagged_stream_block(
            "rg_mapper",
            gr::io_signature::make(1, 1, sizeof(gr_complex)),
            gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
            "packet_len"),
        d_nstrm(nstrm),
        d_n_ofdm_syms(n_ofdm_syms),
        d_sc_num(sc_num),
        d_pilot_sc_ind(pilot_sc_ind),
        d_add_cyclic_shift(addcs),
        d_debug(debug),
        cc(0),
        d_numue(numue),
        d_ueidx(ueidx),
        d_mucpt(mucpt),
        d_add_ltf(addltf),
        d_ltfdata_len(0),
        d_nltfsyms(0),
        d_ltf_data(nullptr),
        d_cpt_pilot(nullptr)
  {
    if (d_debug)
    {
      std::cout << "[CONSTRUCTOR rg_mapper_impl (nstrm=" << nstrm
                << ", n_ofdm_syms=" << n_ofdm_syms
                << ", sc_num=" << sc_num
                << ", addcs=" << addcs
                << ", debug=" << debug
                << ", numue=" << numue
                << ", ueidx=" << ueidx
                << ", mucpt=" << mucpt
                << ", addltf=" << addltf
                << ")]" << std::endl;
    }
    if (nstrm < 1 || nstrm > 4)
    {
      throw std::runtime_error("rg_mapper: Invalid number of streams");
    }

    if (d_n_ofdm_syms < 1 || d_n_ofdm_syms > 200)
      throw std::runtime_error("rg_mapper: Invalid number of data OFDM symbols");
    // Decide FFT size
    if (d_sc_num == 56)
    {
      d_fftsize = 64;
      d_npt = 4;
    }
    else if (d_sc_num == 242)
    {
      d_fftsize = 256;
      d_npt = 8;
    }
    else
    {
      throw std::runtime_error("rg_mapper: Unsupported number of subcarriers");
    }

    // Check pilot array length vs d_npt
    if ((int)d_pilot_sc_ind.size() != d_npt)
    {
      throw std::runtime_error("rg_mapper: pilot_sc_ind.size() must match the expected # of pilot carriers!");
    }

    // The total data symbols per stream
    d_data_syms_per_stream = (d_sc_num - (int)d_pilot_sc_ind.size()) * d_n_ofdm_syms;
    d_total_symbols_required = d_data_syms_per_stream;
    // We require exactly d_data_syms_per_stream input items for each call
    if (d_debug)
    {
      std::cout << "[rg_mapper_impl] data_syms_per_stream=" << d_data_syms_per_stream
                << ", total_symbols_required=" << d_total_symbols_required << std::endl;
    }

    // Prepare for LTF data if requested
    if (d_add_ltf && ltfdata)
    {
      uint64_t data_len = read_ltf_data(ltfdata);
      if (data_len == 0)
      {
        throw std::runtime_error("rg_mapper: Failed to read LTF data or file empty");
      }
      // data_len is total # of gr_complex read
      // Must be multiple of d_nstrm
      if (data_len % d_nstrm != 0)
        throw std::runtime_error("rg_mapper: LTF data length not divisible by # of streams");
      // Then each stream gets data_len/d_nstrm samples
      d_ltfdata_len = data_len / d_nstrm;
      // Must be multiple of d_sc_num
      if (d_ltfdata_len % d_sc_num != 0)
        throw std::runtime_error("rg_mapper: LTF data length for each stream must be multiple of sc_num");
      d_nltfsyms = d_ltfdata_len / d_sc_num;
      if (d_debug)
      {
        std::cout << "[rg_mapper_impl] LTF data loaded successfully: "
                  << d_ltfdata_len << " samples per stream => " << d_nltfsyms
                  << " OFDM symbols for LTF." << std::endl;
      }
    }
    // Pre-compute cyclic phase shifts if d_add_cyclic_shift is used
    // We'll only define for up to 4 streams:
    float cshift[4] = {0.0f, -8.0f, -4.0f, -12.0f};
    for (int ss = 0; ss < 4; ss++)
    {
      d_phaseshift[ss].resize(d_sc_num);
      for (int k = 0; k < d_sc_num / 2; k++)
      {
        int pn = k - d_sc_num / 2;
        int pp = k + 1;
        float angle_n = -2.0f * float(M_PI) * cshift[ss] / float(d_fftsize) * float(pn);
        float angle_p = -2.0f * float(M_PI) * cshift[ss] / float(d_fftsize) * float(pp);
        d_phaseshift[ss][k] = std::exp(gr_complex(0, angle_n));
        d_phaseshift[ss][k + (d_sc_num / 2)] = std::exp(gr_complex(0, angle_p));
      }
    }
    // Pre-generate CPT pilots for the entire data frame
    d_cpt_pilot = (float *)volk_malloc(sizeof(float) * d_npt * d_nstrm * d_n_ofdm_syms, volk_get_alignment());
    generate_cpt_pilots();
    // For safety, ensure the scheduler calls us with at least the entire frame
    set_output_multiple(d_sc_num * d_n_ofdm_syms + d_ltfdata_len);
    set_min_output_buffer(0, d_sc_num * d_n_ofdm_syms + d_ltfdata_len);
    set_tag_propagation_policy(block::TPP_DONT);
  }

  rg_mapper_impl::~rg_mapper_impl()
  {
    if (d_ltf_data)
    {
      volk_free(d_ltf_data);
      d_ltf_data = nullptr;
    }
    if (d_cpt_pilot)
    {
      volk_free(d_cpt_pilot);
      d_cpt_pilot = nullptr;
    }
  }

  // We rely on the base-class tagged_stream_block to chunk the input
  // so that each invocation of work() sees exactly "framelen" items.
  int rg_mapper_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
  {
    // If we get at least 'framelen' items on input, we'll produce d_sc_num*d_n_ofdm_syms
    if (ninput_items[0] >= d_data_syms_per_stream)
      return d_sc_num * d_n_ofdm_syms;
    else
      return 0;
  }

  void rg_mapper_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
  {
    // We want exactly 'd_data_syms_per_stream' input items for one chunk
    // that produces d_sc_num*d_n_ofdm_syms output items.
    ninput_items_required[0] = d_data_syms_per_stream;
  }

  int rg_mapper_impl::work(int noutput_items,
                              gr_vector_int &ninput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
  {
    cc++;
    if (d_debug)
    {
      std::cout << "\n[rg_mapper_impl::work(" << cc << ")] Called, noutput_items="
                << noutput_items << ", ninput_items[0]=" << ninput_items[0] << std::endl;
    }
    // We expect exactly d_data_syms_per_stream input items for one chunk
    int in_count = ninput_items[0];
    if (in_count < d_data_syms_per_stream)
    {
      // Not enough for a full frame => produce nothing
      if (d_debug)
      {
        std::cerr << " [rg_mapper_impl::work(" << cc << ")] Not enough input for a frame: "
                  << in_count << " < " << d_data_syms_per_stream << std::endl;
      }
      return 0;
    }

    // We'll produce (d_sc_num*d_n_ofdm_syms + d_ltfdata_len) items on each output port
    if (noutput_items < (d_sc_num * d_n_ofdm_syms))
    {
      // Not enough output space => produce nothing
      return 0;
    }

    std::vector<gr::tag_t> tags;
    get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("seqno"));
    if (tags.size() == 0)
    {
      throw std::runtime_error("rg_mapper: No seqno tag found");
    }
    d_seqno = pmt::to_uint64(tags[0].value);

    // Input data are complex QAM symbols
    auto in_ptr = static_cast<const gr_complex *>(input_items[0]);
    // if (d_debug && ninput_items[0] >= d_data_syms_per_stream)
    // {
    //   std::cerr << " [rg_mapper_impl::work(" << cc << ")] First 5 input items:\n  ";
    //   for (int i = 0; i < 5; i++)
    //     std::cerr << in_ptr[i] << "  ";
    //   std::cerr << std::endl;
    // }

    // Each output port
    std::vector<gr_complex *> outs(d_nstrm);
    for (int ss = 0; ss < d_nstrm; ss++)
    {
      outs[ss] = static_cast<gr_complex *>(output_items[ss]);
    }

    // 1) If LTF is enabled, prepend it
    int output_offset = 0;
    if (d_add_ltf && d_ltfdata_len > 0)
    {
      // Copy LTF data for each stream
      for (int s = 0; s < d_nstrm; s++)
      {
        std::memcpy((void *)&outs[s][0],                    // output start
                    (void *)&d_ltf_data[s * d_ltfdata_len], // LTF chunk for this stream
                    sizeof(gr_complex) * d_ltfdata_len);
      }
      output_offset = d_ltfdata_len;
    }
    // 2) Map the chunk of input QAM symbols into subcarriers, skipping pilot positions
    //    and inserting the pre-generated CPT pilots.  We'll place them after the LTF area.
    // For each stream, we start reading from in_ptr[ss],
    // then jump by d_nstrm each time we place one data subcarrier.
    for (int ss = 0; ss < d_nstrm; ss++)
    {
      gr_complex *out_s = outs[ss];
      int ptidx = 0;
      int inp_offset = ss; // <--- Key: start at ss
      for (int cursym = 0; cursym < d_n_ofdm_syms; cursym++)
      {
        for (int scidx = 0; scidx < d_sc_num; scidx++)
        {
          int out_pos = output_offset + cursym * d_sc_num + scidx;

          bool is_pilot = false;
          for (auto psc : d_pilot_sc_ind)
          {
            if (scidx == psc)
            {
              is_pilot = true;
              break;
            }
          }

          if (is_pilot)
          {
            // Insert CPT pilot
            int pilot_pos = d_nstrm * d_npt * cursym + ss * d_npt + ptidx;
            out_s[out_pos] = d_cpt_pilot[pilot_pos];
            ptidx = (ptidx + 1) % d_npt;
          }
          else
          {
            // Pull from in_ptr, stepping by d_nstrm
            out_s[out_pos] = in_ptr[inp_offset];
            inp_offset += d_nstrm;
          }
        }
      }
    }
    // 3) Apply cyclic shifts if requested
    if (d_add_cyclic_shift)
    {
      for (int ss = 1; ss < d_nstrm; ss++)
      {
        gr_complex *out_s = outs[ss];
        for (int cursym = 0; cursym < (d_nltfsyms + d_n_ofdm_syms); cursym++)
        {
          for (int k = 0; k < d_sc_num; k++)
          {
            int offset = cursym * d_sc_num + k;
            out_s[offset] *= d_phaseshift[ss][k];
          }
        }
      }
    }

    // 4) Attach packet_len tag on each output port
    int total_out_count = d_sc_num * d_n_ofdm_syms + d_ltfdata_len;
    for (int ss = 0; ss < d_nstrm; ss++)
    {
      add_item_tag(ss, nitems_written(ss), pmt::string_to_symbol("packet_len"), pmt::from_long(total_out_count));
      add_item_tag(ss, nitems_written(ss), pmt::string_to_symbol("seqno"), pmt::from_uint64(d_seqno));
    }

    // if (d_debug)
    // {
    //   // Write the 2D resource grid to CSV
    //   for (int ss = 0; ss < d_nstrm; ss++)
    //   {
    //     std::ofstream ofs("/tmp/rg_" + std::to_string(ss) + ".csv");
    //     for (int sym = 0; sym < d_n_ofdm_syms; sym++)
    //     {
    //       for (int sc = 0; sc < d_sc_num; sc++)
    //       {
    //         auto val = static_cast<gr_complex *>(output_items[ss])[sym * d_sc_num + sc];
    //         ofs << val.real() << ":" << val.imag();
    //         if (sc < d_sc_num - 1)
    //           ofs << ",";
    //       }
    //       ofs << "" << std::endl;
    //     }
    //     ofs.close();
    //     std::cout << " [rg_mapper_impl::work(" << cc << ")] Stored resource grid for stream " << ss << " in /tmp/rg_" << ss << ".csv" << std::endl;
    //   }
    // }

    if (d_debug)
    {
      std::cout << "[rg_mapper_impl::work(" << cc
                << ")] => produced " << total_out_count
                << " items on each of " << d_nstrm << " output streams.\n";
    }
    return total_out_count;
  }

  // ------------------------------------------------------------------------
  // Generate pilot tones for entire OFDM frame the same way as rg_mapper_impl.
  //   - Use an LSFR for pilot parity
  //   - For multi-user mode (d_mucpt, d_ueidx >= 0),
  //       * index the base pilot with sidx = (ueidx*d_nstrm + s)
  //       * zero out for any symbol that doesn't belong to this UE
  // ------------------------------------------------------------------------
  void rg_mapper_impl::generate_cpt_pilots()
  {
    // LSFR approach for pilot parity
    unsigned pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x71; // offset=3 or 4

    // Same base pilot arrays as rg_mapper_impl
    static const float basePilots4[6][4] = {
        {1, 1, -1, -1}, // for 2-stream or 4-stream
        {1, -1, -1, 1},
        {1, 1, 1, -1},
        {1, 1, -1, 1},
        {1, -1, 1, 1},
        {-1, 1, 1, 1}};
    static const float basePilots8[4][8] = {
        {1, 1, 1, -1, -1, 1, 1, 1},
        {1, 1, 1, -1, -1, 1, 1, 1},
        {1, 1, 1, -1, -1, 1, 1, 1},
        {1, 1, 1, -1, -1, 1, 1, 1},
    };

    // For 64-FFT up to 4 streams x up to 2 UEs => up to row 7 in basePilots4
    // but we define 6 rows here.  Typically for 2 UEs x 2 streams => sidx in [0..3].
    // You can extend if you need more.

    for (int symidx = 0; symidx < d_n_ofdm_syms; symidx++)
    {
      // Update pilot parity for current symbol
      unsigned parity_bit = ((pilot_lsfr >> 6) & 1) ^ ((pilot_lsfr >> 3) & 1);
      pilot_lsfr = ((pilot_lsfr << 1) & 0x7E) | parity_bit;
      float pilot_parity = (parity_bit == 1) ? -1.0f : 1.0f;

      // Fill pilot for each pilot subcarrier i
      for (int i = 0; i < d_npt; i++)
      {
        unsigned idx = (symidx + i) % d_npt; // offset in base pilot
        for (int s = 0; s < d_nstrm; s++)
        {
          int offset = d_nstrm * d_npt * symidx + s * d_npt + i;

          // If multi-user => zero out if not our symbol
          if (d_mucpt && d_ueidx >= 0 && ((symidx % d_numue) != d_ueidx))
          {
            d_cpt_pilot[offset] = 0.0f;
            continue;
          }

          // Next find which pilot row to use
          int sidx = (d_mucpt && d_ueidx >= 0) ? (d_ueidx * d_nstrm + s) : s;
          // For 64-FFT => use basePilots4
          // For 256-FFT => use basePilots8
          if (d_fftsize == 64)
          {
            // clamp sidx if needed
            if (sidx >= 6)
              sidx = 5;
            // idx < 4 for 64-FFT
            d_cpt_pilot[offset] = pilot_parity * basePilots4[sidx][idx];
          }
          else
          {
            // d_fftsize=256 => single-stream pattern or replicate
            int base_sidx = (s < 4) ? s : 3; // keep it safe
            // idx < 8
            d_cpt_pilot[offset] = pilot_parity * basePilots8[base_sidx][idx];
          }
        }
      }
    }
  }

  uint64_t rg_mapper_impl::read_ltf_data(const char *filename)
  {
    FILE *fp = std::fopen(filename, "rb");
    if (!fp)
    {
      std::cerr << "[rg_mapper_impl::read_ltf_data] Failed to open " << filename << "\n";
      return 0ULL;
    }
    std::fseek(fp, 0, SEEK_END);
    long fsize = std::ftell(fp);
    if (fsize <= 0)
    {
      std::fclose(fp);
      return 0ULL;
    }
    std::fseek(fp, 0, SEEK_SET);

    uint64_t nitems = (uint64_t)fsize / sizeof(gr_complex);
    if (nitems == 0)
    {
      std::fclose(fp);
      return 0ULL;
    }

    d_ltf_data = (gr_complex *)volk_malloc(sizeof(gr_complex) * nitems, volk_get_alignment());
    size_t nr = std::fread(d_ltf_data, sizeof(gr_complex), nitems, fp);
    std::fclose(fp);

    if (nr != nitems)
    {
      std::cerr << "[rg_mapper_impl::read_ltf_data] Incomplete file read.\n";
      volk_free(d_ltf_data);
      d_ltf_data = nullptr;
      return 0ULL;
    }
    return nitems;
  }

} // namespace gr::ncjt
