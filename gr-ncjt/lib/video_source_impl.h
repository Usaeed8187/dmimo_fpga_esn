/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H
#define INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H

#include <gnuradio/ncjt/video_source.h>
#include <boost/circular_buffer.hpp>

namespace gr::ncjt
{

class video_source_impl : public video_source
{
private:
    const int d_hdr_len = 8;  // packet header length (bytes)
    const int d_segment_len = 188;  // TS segment size (bytes)
    int d_data_frame_len;  // data frame length (bytes)
    int d_max_payload_len;  // maximum payload length (bytes)
    bool d_byteoutput; // output bytes instead of bits

    uint8_t d_hdr_bytes[8]; // buffer fo header bytes
    boost::circular_buffer<uint8_t> *d_tsdata_buffer;
    boost::mutex fp_mutex;

    uint64_t d_frame_cnt; // for debugging
    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    process_pdu_message(const pmt::pmt_t &msg);

public:
    video_source_impl(int framelen, bool byteoutput, bool debug);
    ~video_source_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_VIDEO_SOURCE_IMPL_H */
