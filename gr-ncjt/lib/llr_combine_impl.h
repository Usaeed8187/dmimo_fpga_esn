/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_LLR_COMBINE_IMPL_H
#define INCLUDED_NCJT_LLR_COMBINE_IMPL_H

#include <gnuradio/ncjt/llr_combine.h>

namespace gr::ncjt
{

class llr_combine_impl : public llr_combine
{
private:
    const int SD_NUM = 52; // number of data subcarriers
    int d_nstrm; // number of streams
    int d_modtype1; // modulation order
    int d_modtype2; // modulation order

    float *d_llr_data; // binary data for 6-bit LLR values

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    llr_combine_impl(int nstrm, int modtype1, int modtype2, int blocksize, bool debug);
    ~llr_combine_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_LLR_COMBINE_IMPL_H */
