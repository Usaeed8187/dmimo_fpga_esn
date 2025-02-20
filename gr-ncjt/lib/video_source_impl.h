/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H
#define INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H

#include <gnuradio/ncjt/video_source.h>

namespace gr::ncjt
{

class video_source_impl : public video_source
{
private:
    const int d_hdr_len = 8; // packet header length (bytes)
    const int d_segment_len = 188; // TS segment size (bytes)
    int d_framelen; // fixed frame length in bytes
    int d_payload_len; // maximum payload length

    int d_frame_cnt;
    bool d_debug;

public:
    video_source_impl(int framelen, bool debug);
    ~video_source_impl();

    void
    forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int
    general_work(int noutput_items, gr_vector_int &ninput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H */
