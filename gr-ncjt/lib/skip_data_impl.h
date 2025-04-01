/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_SKIP_DATA_IMPL_H
#define INCLUDED_NCJT_SKIP_DATA_IMPL_H

#include <gnuradio/ncjt/skip_data.h>

namespace gr::ncjt
{

class skip_data_impl : public skip_data
{
private:
    int d_nstrm; // number of streams
    int d_skip_len; // number of samples to skip

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    skip_data_impl(int nstrm, int skiplen, bool debug);
    ~skip_data_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SKIP_DATA_IMPL_H */
