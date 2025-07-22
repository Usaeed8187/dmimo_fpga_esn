/* -*- c++ -*- */
/*
 * Copyright 2025.
 *
 * A “no-air” block that:
 *   - receives 56×(ndatasyms) complex samples from rg_mapper (each “packet”),
 *   - optionally adds AWGN to reach a desired SNR (dB),
 *   - discards pilot subcarriers (indices 7,21,34,48 in each symbol),
 *   - and outputs 52×(ndatasyms) complex samples on out0 (noisy data) and
 *     52×(ndatasyms) complex samples of CSI on out1 (currently unity).
 *
 * The parameter 'frame_per_sec' throttles the flowgraph by sleeping between frames
 * if > 0. If snr_db > 0, we measure the average signal power of the block of input
 * samples, and inject noise accordingly.  If snr_db <= 0, we skip adding noise.
 */

#include "noair_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/tags.h>
#include <cstring>  // for memcpy
#include <iostream> // for debug prints
#include <thread>   // for sleep_for
#include <chrono>   // for steady_clock
#include <cmath>    // for pow, sqrt, etc.
#include <gnuradio/ncjt/rg_modes.h>
#include "common.h"

namespace gr
{
  namespace ncjt
  {

    // ------------------ public make() ------------------
    noair::sptr
    noair::make(int phase,
                int rgmode,
                float frame_per_sec,
                float snr_db,
                int num_drop_init_packets,
                bool debug)
    {
      return gnuradio::make_block_sptr<noair_impl>(phase,
                                                   rgmode,
                                                   frame_per_sec,
                                                   snr_db,
                                                   num_drop_init_packets,
                                                   debug);
    }

    // ------------------ noair_impl methods ------------------

    noair_impl::noair_impl(int phase,
                           int rgmode,
                           float frame_per_sec,
                           float snr_db,
                           int num_drop_init_packets,
                           bool debug)
        : gr::tagged_stream_block("noair",
                                  gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                  gr::io_signature::make(2, 2, sizeof(gr_complex)),
                                  "packet_len"),
          d_debug(debug),
          d_phase(phase),
          d_num_drop_init_packets(num_drop_init_packets),
          d_frame_per_sec(frame_per_sec),
          d_snr_db(snr_db),
          d_norm(0.0f, 1.0f)
    {
      cc = 0;
      // Build a unique ID for attaching tags, debug prints, etc.
      std::stringstream strm;
      strm << name() << unique_id();
      d_me = pmt::string_to_symbol(strm.str());

      set_tag_propagation_policy(TPP_DONT);

      if (d_debug)
      {
        std::cout << "\n[noair_impl] Constructor: rgmode=" << rgmode
                  << ", frame_per_sec=" << d_frame_per_sec
                  << ", snr_db=" << d_snr_db
                  << ", debug=" << d_debug
                  << std::endl;
      }

      if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");
      else
      {
        d_ofdm_syms = RG_NUM_OFDM_SYM[rgmode];
        d_sc_num = RG_NUM_VALID_SC[rgmode];
        for (int k = 0; k < RG_NUM_CPT[rgmode]; k++)
        {
          d_pilot_sc_ind.push_back(RG_CPT_INDX[rgmode][k]);
        }
      }

      // If throttling is enabled (frame_per_sec > 0), compute the time per frame.
      if (d_frame_per_sec > 0.0f)
      {
        d_seconds_per_frame = 1.0 / d_frame_per_sec;
        d_last_frame_time = std::chrono::steady_clock::now();
      }
      else
      {
        d_seconds_per_frame = 0.0;
      }

      // Initialize random engine
      std::random_device rd;
      d_rng.seed(rd());

      message_port_register_in(pmt::intern("s_snr"));
      set_msg_handler(pmt::intern("s_snr"), [this](pmt::pmt_t msg)
                      {
                        if (pmt::is_number(msg))
                        {
                          float new_snr = pmt::to_double(msg);
                            d_snr_db = new_snr;
                            std::cout << "[noair_impl] Updated snr_db to " << d_snr_db << std::endl;
                        } });
    }

    noair_impl::~noair_impl()
    {
    }

    int noair_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      // We expect sc_num * ofdm_syms input. We produce (sc_num - #pilots) * ofdm_syms on each output port.
      const int in_count = ninput_items[0];
      const int want_in = d_sc_num * d_ofdm_syms;
      const int out_count = (d_sc_num - d_pilot_sc_ind.size()) * d_ofdm_syms;

      // If we don't have a full packet, produce 0:
      if (in_count < want_in)
      {
        return 0;
      }
      return out_count;
    }

    int
    noair_impl::work(int noutput_items,
                     gr_vector_int &ninput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
    {
      cc++;
      if (d_debug)
      {
        std::cout << "[noair_impl::work] Called, noutput_items=" << noutput_items << std::endl;
        for (size_t p = 0; p < ninput_items.size(); p++)
        {
          std::cout << " [noair_impl::work(" << cc << ")] input port " << p
                    << " => " << ninput_items[p] << " items" << std::endl;
        }
      }
      // Making sure each work call is "one packet" from upstream.
      const int in_count = ninput_items[0]; // e.g. 2240 for 56×40
      if (in_count == 0)
      {
        if (d_debug)
        {
          std::cout << " [noair_impl::work(" << cc << ")] No input items; returning 0.\n";
        }
        return 0;
      }

      // The expected input size is sc_num×(ofdm_syms). We'll confirm:
      const int expected_input = d_sc_num * d_ofdm_syms;                         // e.g. 2240
      const int output_count = (d_sc_num - d_pilot_sc_ind.size()) * d_ofdm_syms; // e.g. 2080

      // Throttle if frame_per_sec > 0
      if (d_frame_per_sec > 0.0f)
      {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last = std::chrono::duration<double>(now - d_last_frame_time).count();
        if (time_since_last < d_seconds_per_frame)
        {
          double to_sleep = d_seconds_per_frame - time_since_last;
          if (d_debug)
          {
            std::cout << " [noair_impl::work(" << cc << ")] Throttling: sleeping for "
                      << to_sleep << " seconds to maintain "
                      << d_frame_per_sec << " frames/sec.\n";
          }
          std::this_thread::sleep_for(std::chrono::duration<double>(to_sleep));
        }
        // Update last frame time to "now" after we've waited
        d_last_frame_time = std::chrono::steady_clock::now();
      }

      const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
      gr_complex *out0 = reinterpret_cast<gr_complex *>(output_items[0]);
      gr_complex *out1 = reinterpret_cast<gr_complex *>(output_items[1]);

      // Debug tag info
      if (d_debug)
      {
        std::vector<gr::tag_t> pkt_tags;
        get_tags_in_window(pkt_tags, 0, 0, in_count, pmt::string_to_symbol("packet_len"));

        std::cout << " [noair_impl::work(" << cc << ")] input port 0: " << in_count
                  << " items, found " << pkt_tags.size() << " packet_len tags.\n";

        if (!pkt_tags.empty())
        {
          auto &tg = pkt_tags[0];
          long p_len = pmt::to_long(tg.value);
          std::cout << " [noair_impl::work(" << cc << ")] Found input packet_len tag at rel offset "
                    << (tg.offset - nitems_read(0))
                    << " with value=" << p_len << std::endl;
        }
        if (in_count != expected_input)
        {
          std::cout << " [noair_impl::work(" << cc << ")] WARNING: input size " << in_count
                    << " != expected " << expected_input << ".\n";
        }
      }

      std::vector<gr::tag_t> tags;
      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("seqno"));
      if (tags.size() == 0)
      {
        throw std::runtime_error("noair: No seqno tag found");
      }
      d_seqno = pmt::to_uint64(tags[0].value);

      if (d_num_drop_init_packets > 0)
      {
        if (d_seqno < d_num_drop_init_packets)
        {
          return 0;
        }
      }

      // ------------------------------------------------------
      // 1) Measure average power of the entire chunk (all 56 subcarriers).
      // ------------------------------------------------------
      double sum_power = 0.0;
      for (int i = 0; i < in_count; i++)
      {
        float re = in[i].real();
        float im = in[i].imag();
        sum_power += re * re + im * im;
      }
      double avg_power = sum_power / (double)in_count;

      // If snr_db>0, compute noise variance from avg_power / linear_SNR.
      float noise_std = 0.0f;
      if (d_snr_db > 0.0f)
      {
        double lin_snr = std::pow(10.0, (double)d_snr_db / 10.0);
        double noise_var = (avg_power / lin_snr);
        // For complex AWGN, each real/imag is N(0, noise_var/2).
        noise_std = std::sqrt(noise_var / 2.0);
      }

      // ------------------------------------------------------
      // 2) Drop pilot subcarriers, add AWGN if desired
      // ------------------------------------------------------
      int out_idx = 0;
      for (int sym = 0; sym < d_ofdm_syms; sym++)
      {
        int sym_offset_in = sym * d_sc_num;

        for (int sc = 0; sc < d_sc_num; sc++)
        {
          bool is_pilot = false;
          for (int pilot : d_pilot_sc_ind)
          {
            if (sc == pilot)
            {
              is_pilot = true;
              break;
            }
          }
          if (is_pilot)
            continue; // skip these pilot subcarriers in the output

          // Optionally add noise to the data subcarrier
          gr_complex orig_val = in[sym_offset_in + sc];
          gr_complex noisy_val = orig_val;
          if (d_snr_db > 0.0f)
          {
            float nr = d_norm(d_rng) * noise_std;
            float ni = d_norm(d_rng) * noise_std;
            noisy_val += gr_complex(nr, ni);
          }

          // Output #1: actual data subcarrier (with AWGN if snr_db>0)
          out0[out_idx] = noisy_val;
          // Output #2: dummy CSI = 1.0 + 0j
          out1[out_idx] = gr_complex(1.0f, 0.0f);

          out_idx++;
        }
      }
      // out_idx should now be 52×(d_ofdm_syms).

      // ------------------------------------------------------
      // 3) Attach packet_len = 52×(d_ofdm_syms) on both outputs
      // ------------------------------------------------------
      const uint64_t abs_out0 = nitems_written(0);
      const uint64_t abs_out1 = nitems_written(1);

      add_item_tag(0, abs_out0,
                   pmt::string_to_symbol("packet_len"),
                   pmt::from_long(output_count),
                   d_me);

      // SNR tags
      if (d_phase != 1)
      {
        int sd_num = d_sc_num - d_pilot_sc_ind.size();
        std::vector<float> SNRs(sd_num, 0.0f);
        for (int i = 0; i < sd_num; i++)
        {
          SNRs[i] = std::pow(10.0, (double)d_snr_db / 10.0);
        }
        add_item_tag(0, nitems_written(0), pmt::string_to_symbol("snr_sc_linear"),
                     pmt::make_blob(SNRs.data(), SNRs.size() * sizeof(float)),
                     pmt::string_to_symbol(name()));
      }
      ////
      add_item_tag(0, abs_out0,
                   pmt::string_to_symbol("seqno"),
                   pmt::from_uint64(d_seqno),
                   d_me);

      add_item_tag(1, abs_out1,
                   pmt::string_to_symbol("packet_len"),
                   pmt::from_long(output_count),
                   d_me);

      if (d_debug)
      {
        std::cout << " [noair_impl::work(" << cc << ")] => AWGN(SNR=" << d_snr_db
                  << " dB), pilot-dropped => " << output_count
                  << " items on each output.\n";
      }

      // We consumed exactly sc_num×(ofdm_syms) from input, produce 52×(ofdm_syms).
      return output_count;
    }

  } // namespace ncjt
} // namespace gr
