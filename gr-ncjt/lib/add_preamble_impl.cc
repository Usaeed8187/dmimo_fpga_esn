/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "add_preamble_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

add_preamble::sptr
add_preamble::make(int nstrm, const char *filename, bool debug)
{
    return gnuradio::make_block_sptr<add_preamble_impl>(nstrm, filename, debug);
}

add_preamble_impl::add_preamble_impl(int nstrm, const char *filename, bool debug)
    : gr::tagged_stream_block(
    "add_preamble",
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    "packet_len"), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 8)
        throw std::runtime_error("only support 1 to 8 streams");
    d_nstrm = nstrm;

    d_preamble_data = nullptr;
    if ((d_preamble_len = read_preamble_data(filename)) <= 0)
        throw std::runtime_error("failed to read preamble data");
    d_preamble_len /= d_nstrm;
    dout << "Preamble length: " << d_preamble_len << std::endl;

    set_tag_propagation_policy(block::TPP_DONT);
}

add_preamble_impl::~add_preamble_impl()
{
    if (d_preamble_data)
        volk_free(d_preamble_data);
}

int
add_preamble_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int noutput_items = ninput_items[0] + d_preamble_len;
    return noutput_items;
}

int
add_preamble_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    int data_length = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        data_length = std::min(data_length, ninput_items[ch]);
    if (data_length == 0)
        return 0;

    int frame_length = d_preamble_len + data_length;

    for (int s = 0; s < d_nstrm; s++)
    {
        auto in = (const gr_complex *) input_items[s];
        auto out = (gr_complex *) output_items[s];

        add_item_tag(s, nitems_written(s),
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(frame_length),
                     pmt::string_to_symbol(name()));

        add_item_tag(s, nitems_written(s),
                     pmt::string_to_symbol("frame_start"),
                     pmt::from_long(frame_length),
                     pmt::string_to_symbol(name()));

        // copy beacon symbols
        memcpy((void *) &out[0], &d_preamble_data[s * d_preamble_len], sizeof(gr_complex) * d_preamble_len);
        // copy data symbols
        memcpy((void *) &out[d_preamble_len], &in[0], sizeof(gr_complex) * data_length);

    }

    return frame_length;
}

uint64_t
add_preamble_impl::read_preamble_data(const char *filename)
{
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return 0;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t data_len = file_size / sizeof(gr_complex);
    if (data_len == 0)
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    d_preamble_data = (gr_complex *) malloc(file_size);
    if (data_len != fread(d_preamble_data, sizeof(gr_complex), data_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        free(d_preamble_data);
        return 0;
    }

    return data_len;
}

} /* namespace gr::ncjt */
