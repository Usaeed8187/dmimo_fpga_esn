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
video_source::make(int framelen, bool byteoutput, bool debug)
{
    return gnuradio::make_block_sptr<video_source_impl>(framelen, byteoutput, debug);
}

video_source_impl::video_source_impl(int framelen, bool byteoutput, bool debug)
    : gr::tagged_stream_block(
    "video_source",
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    "packet_len"), d_byteoutput(byteoutput), d_debug(debug)
{
    if ((framelen - d_hdr_len) < d_segment_len)
        throw std::runtime_error("data frame length too small");

    d_data_frame_len = framelen;
    d_max_payload_len = ((d_data_frame_len - d_hdr_len) / d_segment_len) * d_segment_len;
    d_frame_cnt = 0;

    d_tsdata_buffer = new boost::circular_buffer<uint8_t>(200 * d_max_payload_len);

    message_port_register_in(pmt::mp("pdus"));
    set_msg_handler(pmt::mp("pdus"), [this](const pmt::pmt_t &msg) { process_pdu_message(msg); });

    set_tag_propagation_policy(TPP_DONT);
}

video_source_impl::~video_source_impl()
{
    delete d_tsdata_buffer;
}

int
video_source_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = d_byteoutput ? d_data_frame_len : (8 * d_data_frame_len);
    return noutput_items;
}

void
video_source_impl::process_pdu_message(const pmt::pmt_t &msg)
{
    gr::thread::scoped_lock lock(fp_mutex);

    pmt::pmt_t tsdata_blob = pmt::cdr(msg);
    int dlen = (int) pmt::blob_length(tsdata_blob) / sizeof(uint8_t);
    // dout << "Received " << dlen << " byte of TS stream data" << std::endl;

    if (dlen == 0)
        return;

    if (d_tsdata_buffer->full())
    {
        dout << "TS data buffer overflow" << std::endl;
        return;
    }

    // Add data to buffer
    auto tsdata = (const uint8_t *) pmt::blob_data(tsdata_blob);
    for (int i = 0; i < dlen; i++)
    {
        d_tsdata_buffer->push_back(tsdata[i]);
    }
}

int
video_source_impl::work(int noutput_items,
                        gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock lock(fp_mutex);

    // auto in = (const uint8_t *) input_items[0];
    auto out = (uint8_t *) output_items[0];
    int output_len = d_byteoutput ? d_data_frame_len : (8 * d_data_frame_len);

    if (noutput_items < output_len)
        throw std::runtime_error("output buffer overflow");

    // check how many TS segments are available
    int input_data_len = d_tsdata_buffer->size();
    int num_segment = (std::min(input_data_len, d_max_payload_len) / d_segment_len);
    input_data_len = num_segment * d_segment_len;

    // 8 bytes header contains data length
    auto const *dlen = reinterpret_cast<unsigned char const *>(&input_data_len);
    if (input_data_len == d_max_payload_len)
        d_hdr_bytes[2] = d_hdr_bytes[1] = d_hdr_bytes[0] = 0x55; // full packet header
    else
        d_hdr_bytes[2] = d_hdr_bytes[1] = d_hdr_bytes[0] = 0xaa; // partial packet header byte
    for (int k = 0; k < 4; k++) // fragment length in bytes
        d_hdr_bytes[k + 3] = dlen[k];
    d_hdr_bytes[7] = crc8(d_hdr_bytes + 3, 4); // CRC8 check

    // clear output memory
    memset(out, 0, output_len * sizeof(uint8_t));

    for (int m = 0; m < d_hdr_len; m++)
    {
        if (d_byteoutput)
            out[m] = d_hdr_bytes[m];
        else
        {
            for (int k = 0; k < 8; k++)
                out[8 * m + k] = (d_hdr_bytes[m] >> k) & 0x01;
        }
    }

    for (int n = 0; n < input_data_len; n++)
    {
        uint8_t cur_byte = d_tsdata_buffer->at(0);
        d_tsdata_buffer->pop_front();
        if (d_byteoutput)
            out[d_hdr_len + n] = cur_byte;
        else
        {
            for (int k = 0; k < 8; k++)
                out[8 * (n + d_hdr_len) + k] = (cur_byte >> k) & 0x01;
        }
    }

    // scramble remaining data bytes
    uint16_t lfsr = 0xACE1u;
    uint16_t shift_bit;
    for (int n = 0; n < output_len; n++)
    {
        if (d_byteoutput)
        {
            uint8_t scramble_bits  = 0u;
            for (int k = 0; k < 8; k++) {
                scramble_bits |= (lfsr & 0x01) << k;
                shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
                lfsr = (lfsr >> 1) | (shift_bit << 15);
            }
            out[n] ^= scramble_bits;
        }
        else
        {
            out[n] ^= (lfsr & 0x01);
            shift_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
            lfsr = (lfsr >> 1) | (shift_bit << 15);
        }
    }

    d_frame_cnt += 1;
    if (d_frame_cnt % 100 == 0)
        dout << "Tx data size: " << input_data_len << std::endl;

    // add packet_len tag for tagged stream blocks
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(output_len),
                 pmt::string_to_symbol(name()));

    return output_len;
}

} /* namespace gr::ncjt */
