/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_VIDEO_SINK_IMPL_H
#define INCLUDED_NCJT_VIDEO_SINK_IMPL_H

#include <gnuradio/ncjt/video_sink.h>

namespace gr {
namespace ncjt {

class video_sink_impl : public video_sink {
private:
    const int d_hdr_len = 8; // packet header length (bytes)
    int d_framelen; // fixed frame length in bytes

    int d_frame_cnt;

    bool d_debug;

protected:
  int calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
  video_sink_impl(int framelen, bool debug);
  ~video_sink_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_VIDEO_SINK_IMPL_H */
