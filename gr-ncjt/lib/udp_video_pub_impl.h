/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UDP_VIDEO_PUB_IMPL_H
#define INCLUDED_NCJT_UDP_VIDEO_PUB_IMPL_H

#include <gnuradio/ncjt/udp_video_pub.h>

namespace gr
{
  namespace ncjt
  {

    class udp_video_pub_impl : public udp_video_pub
    {
    private:
      /* --- video sink --- */
      std::thread d_video_thr;
      std::atomic<bool> d_video_run{false};

      std::deque<std::array<uint8_t, 188>> d_video_fifo;
      std::mutex d_video_mtx;

      /* keep a partial packet between work() calls */
      std::deque<uint8_t> d_pkt_accum;

      std::string d_mcast_grp = "239.255.0.1";
      uint16_t d_mcast_port = 9999;

      void video_loop();
      void start_video_sink();
      void stop_video_sink();

    protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

    public:
      udp_video_pub_impl(const std::string &grp, int port);
      ~udp_video_pub_impl();

      // Where all the action really happens
      int work(int noutput_items, gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_UDP_VIDEO_PUB_IMPL_H */
