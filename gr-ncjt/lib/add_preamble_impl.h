/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_ADD_PREAMBLE_IMPL_H
#define INCLUDED_NCJT_ADD_PREAMBLE_IMPL_H

#include <gnuradio/ncjt/add_preamble.h>

namespace gr::ncjt
{

class add_preamble_impl : public add_preamble
{
private:
    int d_nstrm;          // number of streams
    int d_preamble_len;   // preamble length in samples
    int d_delay;   // extra sample delay

    gr_complex *d_preamble_data;  // preamble data buffer

    const bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    uint64_t
    read_preamble_data(const char *filename);

public:
    add_preamble_impl(int nstrm, const char *filename, int delay, bool debug);
    ~add_preamble_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_ADD_PREAMBLE_IMPL_H */
