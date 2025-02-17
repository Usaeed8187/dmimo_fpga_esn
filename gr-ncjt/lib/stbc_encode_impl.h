/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_STBC_ENCODE_IMPL_H
#define INCLUDED_NCJT_STBC_ENCODE_IMPL_H

#include <gnuradio/ncjt/stbc_encode.h>

namespace gr::ncjt
{

class stbc_encode_impl : public stbc_encode
{
private:
    int d_fftsize; // OFDM FFT size
    int d_scnum; // number of valid subcarriers
    int d_ndatasyms;  // total number of data OFDM symbols)
    int d_npilotsyms; // number of pilot OFDM symbols
    int d_numsyms; // total number of OFDM symbols

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    stbc_encode_impl(int fftsize, int ndatasyms, int npilotsyms, bool debug);
    ~stbc_encode_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_STBC_ENCODE_IMPL_H */
