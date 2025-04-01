/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_STBC_DECODE_IMPL_H
#define INCLUDED_NCJT_STBC_DECODE_IMPL_H

#include <gnuradio/ncjt/stbc_decode.h>
#include "cmatrix.h"

namespace gr::ncjt
{

class stbc_decode_impl : public stbc_decode
{
private:
    const int d_nrx = 2; // number of receive antennas
    int d_scnum; // number of valid subcarriers
    int d_scdata; // number data subcarriers
    int d_ndatasyms;  // total number of data OFDM symbols)
    int d_npilotsyms; // number of pilot OFDM symbols
    int d_numsyms; // total number of OFDM symbols

    float d_cpe_phi; // CPE estimation slope
    float d_cpe_offset; // CPE estimation offset

    gr_complex *d_chan_est; // channel estimation using H-LTFs
    uint8_t *d_llr_data;    // LLR magnitude per data subcarrier

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    std::tuple<CTensor2D, Tensor2D>
    alamouti_decode(const CTensor4D& r, const CTensor4D& h);

    void send_llr_message(void);

public:
    stbc_decode_impl(int fftsize, int ndatasyms, int npilotsyms, bool debug);
    ~stbc_decode_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_STBC_DECODE_IMPL_H */
