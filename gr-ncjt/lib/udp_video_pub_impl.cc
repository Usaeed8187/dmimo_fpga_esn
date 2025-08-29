/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "udp_video_pub_impl.h"
#include <gnuradio/io_signature.h>

#include <random>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace gr
{
  namespace ncjt
  {

    void udp_video_pub_impl::video_loop()
    {
      int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (sock < 0)
      {
        std::cerr << "[udp_video_pub::video_loop] ERROR: Failed to create socket: " << strerror(errno) << std::endl;
        return;
      }

      // Enable multicast loopback for local testing
      int loop = 1;
      if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0)
      {
        std::cerr << "[udp_video_pub::video_loop] WARNING: Failed to set IP_MULTICAST_LOOP: " << strerror(errno) << std::endl;
      }

      // Set TTL for multicast (1 is fine for local testing)
      unsigned char ttl = 1;
      if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0)
      {
        std::cerr << "[udp_video_pub::video_loop] WARNING: Failed to set IP_MULTICAST_TTL: " << strerror(errno) << std::endl;
      }

      // Ensure the multicast group address is valid
      std::string mcast_grp = d_mcast_grp;
      if (mcast_grp == "127.0.0.1")
      {
        mcast_grp = "239.255.0.1"; // Use a valid multicast address
        std::cout << "[udp_video_pub::video_loop] WARNING: Replaced 127.0.0.1 with 239.255.0.1 for multicast" << std::endl;
      }

      sockaddr_in dst{};
      dst.sin_family = AF_INET;
      dst.sin_port = htons(d_mcast_port);

      if (::inet_pton(AF_INET, mcast_grp.c_str(), &dst.sin_addr) <= 0)
      {
        std::cerr << "[demapper_impl::video_loop] ERROR: Invalid multicast address: " << mcast_grp << std::endl;
        ::close(sock);
        return;
      }

      // Print streaming URL for clients
      std::cout << "Video streaming on udp://@" << mcast_grp << ":" << d_mcast_port << std::endl;

      while (d_video_run)
      {
        std::array<uint8_t, 188> pkt;
        {
          std::unique_lock<std::mutex> lk(d_video_mtx);
          if (d_video_fifo.empty())
          {
            lk.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
          }
          pkt = d_video_fifo.front();
          d_video_fifo.pop_front();
        }
        ssize_t sent = ::sendto(sock, pkt.data(), pkt.size(), 0,
                                reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
        if (sent < 0)
        {
          std::cerr << "[demapper_impl::video_loop] WARNING: Failed to send packet: " << strerror(errno) << std::endl;
        }
      }
      ::close(sock);
    }

    void udp_video_pub_impl::start_video_sink()
    {
      d_video_run = true;
      d_video_thr = std::thread([this]
                                { this->video_loop(); });
    }
    void udp_video_pub_impl::stop_video_sink()
    {
      d_video_run = false;
      if (d_video_thr.joinable())
        d_video_thr.join();
    }

    //////////////////////////////////////////////////////////////////////

    udp_video_pub::sptr udp_video_pub::make(const std::string &grp, int port)
    {
      return gnuradio::make_block_sptr<udp_video_pub_impl>(grp, port);
    }

    /*
     * The private constructor
     */
    udp_video_pub_impl::udp_video_pub_impl(const std::string &grp, int port)
        : gr::tagged_stream_block(
              "udp_video_pub",
              gr::io_signature::make(1, 1, sizeof(uint8_t)),
              gr::io_signature::make(1, 1, sizeof(uint8_t)),
              "packet_len")
    {
      d_mcast_port = (uint16_t)port;

      // start video sink thread
      if (d_mcast_port != 0)
        start_video_sink();
    }

    /*
     * Our virtual destructor.
     */
    udp_video_pub_impl::~udp_video_pub_impl()
    {
      stop_video_sink();
    }

    int udp_video_pub_impl::calculate_output_stream_length(
        const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0]; // 1:1 mapping for now
      return noutput_items;
    }

    int udp_video_pub_impl::work(int noutput_items, gr_vector_int &ninput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items)
    {
      auto in = static_cast<const uint8_t *>(input_items[0]);
      auto out = static_cast<uint8_t *>(output_items[0]);
      int nitems = ninput_items[0];

      std::memcpy(out, in, nitems * sizeof(uint8_t));

      ///////

      std::vector<gr::tag_t> tags;

      get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_video_on"));
      bool video_on = false;
      if (tags.size())
      {
        video_on = pmt::to_bool(tags[0].value);
      }

      if (video_on)
      {
        get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_seqno"));
        uint16_t seqno = pmt::to_uint64(tags[0].value);

        get_tags_in_window(tags, 0, 0, 1, pmt::string_to_symbol("rx_modtype_phase2"));
        if (!tags.size())
          throw std::runtime_error(
              "[remapper_muxer_impl] ERROR: The rx_modtype_phase2 tag was not found in input stream");
        int phase2_modtype = pmt::to_uint64(tags[0].value);

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

        std::mt19937 gen_bits(1337);
        std::uniform_int_distribution<int> dist_bit(0, 1);

        // --- video streaming mode: push TS packets into the multicast fifo ---
        int bits_per_byte = 8;
        int total_bytes = in_bits_needed / bits_per_byte;
        std::vector<uint8_t> decoded_bytes(total_bytes);
        for (int i = 0; i < total_bytes; i++)
        {
          uint8_t v = 0;
          for (int b = 0; b < bits_per_byte; b++)
          {
            uint8_t bit = (raw_in_bits[i * bits_per_byte + b] & 0x1);
            // unscramble bit BEFORE reconstruction
            bit ^= (dist_bit(gen_bits) & 0x1);
            v = (v << 1) | bit;
          }
          decoded_bytes[i] = v;
        }

        d_pkt_accum.insert(d_pkt_accum.end(),
                           decoded_bytes.begin(),
                           decoded_bytes.end());

        while (d_pkt_accum.size() >= 188)
        {
          /* --- byte-sync: look for a valid 0x47 sync byte ----------------- */
          if (d_pkt_accum[0] != 0x47)
          {                          // not aligned yet
            d_pkt_accum.pop_front(); // discard one byte, try again
            continue;
          }
          /* optional second check – avoids false positives on random data */
          if (d_pkt_accum.size() >= 376 && d_pkt_accum[188] != 0x47)
          {
            d_pkt_accum.pop_front(); // lone 0x47, keep searching
            continue;
          }

          std::array<uint8_t, 188> pkt;
          std::copy_n(d_pkt_accum.begin(), 188, pkt.begin());
          d_pkt_accum.erase(d_pkt_accum.begin(), d_pkt_accum.begin() + 188);

          // ------------------------------------------------------------------
          // Detect MPEG-TS null packet (PID 0x1FFF) and drop it for the video
          // sink, but *do not* drop it from the decoded output stream.
          // Bytes:  [0]  0x47  sync
          //         [1]  xxxx x???   upper 5 bits of PID
          //         [2]  lower 8 bits of PID
          // PID = ((byte1 & 0x1F) << 8) | byte2
          // ------------------------------------------------------------------
          uint16_t pid = ((pkt[1] & 0x1F) << 8) | pkt[2];
          if (pid == 0x1FFF)
          {
            // It’s a null packet – just ignore for the video thread
            continue;
          }

          // Anything else is a “real” packet → deliver to the video sink
          {
            std::lock_guard<std::mutex> lk(d_video_mtx);
            d_video_fifo.push_back(pkt);
          }
        }

        // tell downstream where to pick up the multicast stream
        std::string url = std::string("udp://") + d_mcast_grp + ":" +
                          std::to_string(d_mcast_port);
        auto name = pmt::string_to_symbol(this->name());
        add_item_tag(0, nitems_written(0),
                     pmt::string_to_symbol("rx_udp_stream"),
                     pmt::string_to_symbol(url),
                     name);
      }

      return nitems;
    }

  } /* namespace ncjt */
} /* namespace gr */
