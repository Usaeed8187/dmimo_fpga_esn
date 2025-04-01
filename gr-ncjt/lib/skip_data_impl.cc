/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "skip_data_impl.h"
#include <gnuradio/io_signature.h>

namespace gr::ncjt
{

skip_data::sptr
skip_data::make(int nstrm, int skiplen, bool debug)
{
    return gnuradio::make_block_sptr<skip_data_impl>(nstrm, skiplen, debug);
}

skip_data_impl::skip_data_impl(int nstrm, int skiplen, bool debug)
    : gr::tagged_stream_block(
    "skip_data",
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    "packet_len")
{
    d_nstrm = nstrm;
    if (skiplen <= 0)
        throw std::runtime_error("invalid skip length specified");
    d_skip_len = skiplen;
}

skip_data_impl::~skip_data_impl()
{
}

int
skip_data_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    return ninput_items[0] - d_skip_len;
}

int
skip_data_impl::work(int noutput_items, gr_vector_int &ninput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    int num_output_items = min_input_items - d_skip_len;

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        auto in = static_cast<const gr_complex *>(input_items[ch]);
        auto out = static_cast<gr_complex *>(output_items[ch]);

        memcpy((void *) &out[0], &in[d_skip_len], sizeof(gr_complex) * num_output_items);
    }

    return num_output_items;
}

} /* namespace gr::ncjt */
