/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "video_sink_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

video_sink::sptr
video_sink::make(int framelen, bool debug)
{
    return gnuradio::make_block_sptr<video_sink_impl>(framelen, debug);
}

video_sink_impl::video_sink_impl(int framelen, bool debug)
    : gr::tagged_stream_block(
    "video_sink",
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    "packet_len"), d_debug(debug)
{
    if (framelen < 100 || framelen >= 32768)
        throw std::runtime_error("invalid frame length");
    d_framelen = framelen;

    set_tag_propagation_policy(block::TPP_DONT);
}

video_sink_impl::~video_sink_impl() {}

int
video_sink_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_framelen - d_hdr_len;
    return noutput_items;
}

int
video_sink_impl::work(int noutput_items, gr_vector_int &ninput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    auto in = static_cast<const uint8_t *>(input_items[0]);
    auto out = static_cast<uint8_t *>(output_items[0]);

    if (ninput_items[0] != d_framelen)
    {
        dout << "Input buffer size: " << ninput_items[0] << std::endl;
        throw std::runtime_error("receive frame length incorrect");
    }

    //int data_len = in[3] + 256 * in[4] + 65536 * in[5];

    /*if (in[7] == crc8(&in[3], 4) && data_len <= 32768)
    {
        // copy data to output buffer
        memcpy(&out[0], &in[d_hdr_len], sizeof(uint8_t) * data_len);
    }
    else
    {
        dout << "Payload size: " << data_len << " bytes)" << std::endl;
        dout << "CRC error or invalid payload size" << std::endl;
        data_len = 0;
    }*/

    memcpy(&out[0], &in[d_hdr_len], sizeof(uint8_t) * d_framelen);

    /*d_frame_cnt += 1;
    if (d_frame_cnt % 100 == 0)
        dout << "Rx data size: " << data_len << std::endl;*/

    return d_framelen;
}

} /* namespace gr::ncjt */
