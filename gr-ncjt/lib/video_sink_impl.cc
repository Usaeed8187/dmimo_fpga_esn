/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "video_sink_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

video_sink::sptr
video_sink::make(int framelen, bool byteinput, bool debug)
{
    return gnuradio::make_block_sptr<video_sink_impl>(framelen, byteinput, debug);
}

video_sink_impl::video_sink_impl(int framelen, bool byteinput, bool debug)
    : gr::tagged_stream_block(
    "video_sink",
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    "packet_len"), d_byteinput(byteinput), d_debug(debug)
{
    if (framelen < 100 || framelen >= 32768)
        throw std::runtime_error("invalid frame length");

    d_data_frame_len = framelen;
    d_max_payload_len = ((d_data_frame_len - d_hdr_len) / d_segment_len) * d_segment_len;
    d_frame_cnt = 0;

    set_tag_propagation_policy(block::TPP_DONT);
}

video_sink_impl::~video_sink_impl() {}

int
video_sink_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_data_frame_len;
    return noutput_items;
}

int
video_sink_impl::work(int noutput_items, gr_vector_int &ninput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    auto in = static_cast<const uint8_t *>(input_items[0]);
    auto out = static_cast<uint8_t *>(output_items[0]);
    int input_len = d_byteinput ? d_data_frame_len : (8 * d_data_frame_len);
    if (ninput_items[0] != input_len)
        throw std::runtime_error("received data frame with incorrect length");

    if (noutput_items < d_data_frame_len)
        return 0;

    // data de-scrambling for header bytes
    uint16_t lfsr = 0xACE1u;
    uint16_t shift_bit;
    if (d_byteinput)
    {
        for (int n = 0; n < d_hdr_len; n++)
        {
            uint8_t scramble_bits  = 0u;
            for (int k = 0; k < 8; k++) {
                scramble_bits |= (lfsr & 0x01) << k;
                shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
                lfsr = (lfsr >> 1) | (shift_bit << 15);
            }
            d_hdr_bytes[n] = in[n] ^ scramble_bits;
        }
    }
    else
    {
        for (int n = 0; n < d_hdr_len; n++)
        {
            uint8_t cur_byte = 0;
            for (int k = 0; k < 8; k++)
            {
                cur_byte += ((in[8 * n + k] ^ (lfsr & 0x01)) << k);
                shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
                lfsr = (lfsr >> 1) | (shift_bit << 15);
            }
            d_hdr_bytes[n] = cur_byte;
        }
    }

    int data_len = d_hdr_bytes[3] + 256 * d_hdr_bytes[4] + 65536 * d_hdr_bytes[5];
    if (d_hdr_bytes[7] != crc8(&d_hdr_bytes[3], 4))
    {
        dout << "Payload size CRC error" << std::endl;
        return 0;
    }
    if (data_len > d_max_payload_len)
    {
        dout << "Invalid payload size: " << data_len << " bytes)" << std::endl;
        return 0;
    }
    if (data_len == 0)
        return 0;

    // copy data to output buffer
    if (d_byteinput)
    {
        for (int n = 0; n < data_len; n++)
        {
            uint8_t scramble_bits  = 0u;
            for (int k = 0; k < 8; k++) {
                scramble_bits |= (lfsr & 0x01) << k;
                shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
                lfsr = (lfsr >> 1) | (shift_bit << 15);
            }
            out[n] = in[n + d_hdr_len] ^ scramble_bits;
        }
    }
    else
    {
        for (int n = 0; n < data_len; n++)
        {
            uint8_t cur_byte = 0;
            for (int k = 0; k < 8; k++)
            {
                uint8_t cur_bit = in[8 * (n + d_hdr_len) + k] ^ (lfsr & 0x01);
                cur_byte += (cur_bit << k);
                shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
                lfsr = (lfsr >> 1) | (shift_bit << 15);
            }
            out[n] = cur_byte;
        }
    }

    d_frame_cnt += 1;
    if (d_frame_cnt % 100 == 0)
        dout << "Rx data size: " << data_len << std::endl;

    // add packet_len tag for tagged stream blocks
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(data_len),
                 pmt::string_to_symbol(name()));

    return data_len;
}

} /* namespace gr::ncjt */
