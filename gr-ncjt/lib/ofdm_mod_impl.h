/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_OFDM_MOD_IMPL_H
#define INCLUDED_NCJT_OFDM_MOD_IMPL_H

#include <gnuradio/ncjt/ofdm_mod.h>
#include <gnuradio/fft/fft.h>

namespace gr::ncjt
{

class ofdm_mod_impl : public ofdm_mod
{
private:
    int d_fftsize; // OFDM FFT size
    int d_cplen; // OFDM cyclic prefix length
    int d_symlen; // SYM_LEN = FFT_SIZE + CP_LEN;
    int d_scnum; // SC_NUM number of valid subcarriers
    int d_scnum_half; // SC_NUM_HALF = SC_NUM/2

    float d_scaling; // IQ sample scaling
    fft::fft_complex_rev d_ifft;
    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    ofdm_mod_impl(int fftsize, int cplen, int ntx, float scaling, bool debug);
    ~ofdm_mod_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_OFDM_MOD_IMPL_H */
