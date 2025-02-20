/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "video_source_impl.h"
#include <gnuradio/io_signature.h>

#include "utils.h"

namespace gr::ncjt
{

video_source::sptr
video_source::make(int framelen, bool debug)
{
    return gnuradio::make_block_sptr<video_source_impl>(framelen, debug);
}

video_source_impl::video_source_impl(int framelen, bool debug)
    : gr::block("video_source",
                gr::io_signature::make(1, 1, sizeof(uint8_t)),
                gr::io_signature::make(1, 1, sizeof(uint8_t))),
      d_debug(debug)
{
    if ((framelen - d_hdr_len) < d_segment_len)
        throw std::runtime_error("frame length too small");

    d_framelen = framelen;
    d_payload_len = ((d_framelen - d_hdr_len) / d_segment_len) * d_segment_len;
    d_frame_cnt = 0;

    set_tag_propagation_policy(TPP_DONT);
}

video_source_impl::~video_source_impl() {}

void
video_source_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    /*if (noutput_items >= d_framelen)
    {
        ninput_items_required[0] = d_payload_len;
    }
    else
    {
        ninput_items_required[0] = 0;
    }*/
    ninput_items_required[0] = d_payload_len;
}

int
video_source_impl::general_work(int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
{
    auto in = (const uint8_t *)input_items[0];
    auto out = (uint8_t *) output_items[0];

    if (noutput_items < d_framelen || ninput_items[0] < d_framelen)
        return 0;

    /*int num_segment = (std::min(ninput_items[0], d_payload_len) / d_segment_len);
    if (num_segment == 0)
        return 0;*/

    // int input_data_len = num_segment * d_segment_len;
    int input_data_len = d_framelen;

    // 8 bytes header contains data length
    /* auto const *dlen = reinterpret_cast<unsigned char const *>(&input_data_len);
    if (input_data_len == d_segment_len)
        out[2] = out[1] = out[0] = 0x55; // full packet header
    else
        out[2] = out[1] = out[0] = 0xaa; // partial packet header byte
    for (int k = 0; k < 4; k++) // fragment length in bytes
        out[k + 3] = dlen[k];
    out[7] = crc8(&out[3], 4); // CRC8 check
    */

    // copy data to output buffer
    memcpy(&out[0], &in[0], sizeof(uint8_t) * input_data_len);

    // scramble remaining data bytes
    /*uint16_t lfsr = 0xACE1u;
    uint16_t shift_bit;
    int offset = input_data_len + d_hdr_len;
    while (offset < d_framelen)
    {
        out[offset] ^= (lfsr & 0xff);
        shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
        lfsr = (lfsr >> 1) | (shift_bit << 15);
        offset += 1;
    }*/

    /*d_frame_cnt += 1;
    if (d_frame_cnt % 100 == 0)
        dout << "Tx data size: " << input_data_len << std::endl;*/

    // add packet_len tag for tagged stream blocks
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(d_framelen),
                 pmt::string_to_symbol(name()));

    consume(0, input_data_len);
    return d_framelen;
}

} /* namespace gr::ncjt */
