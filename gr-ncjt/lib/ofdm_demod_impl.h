/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_OFDM_DEMOD_IMPL_H
#define INCLUDED_NCJT_OFDM_DEMOD_IMPL_H

#include <gnuradio/fft/fft.h>
#include <gnuradio/ncjt/ofdm_demod.h>

namespace gr::ncjt
{

class ofdm_demod_impl : public ofdm_demod
{
private:
    int d_fftsize; // OFDM FFT size
    int d_cplen; // OFDM cyclic prefix length
    int d_symlen; // SYM_LEN = FFT_SIZE + CP_LEN;
    int d_scnum; // number of valid subcarriers
    int d_scnum_half; // SC_NUM_HALF = SC_NUM/2
    int d_left_guard_scnum; // number of left guard subcarriers
    int d_center_null_scnum_pos_half; // number of center null subcarriers on the positive half subcarriers
    int d_symbol_offset;     // OFDM symbol offset
    fft::fft_complex_fwd d_fft;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    ofdm_demod_impl(int fftsize, int cplen, int symoffset);
    ~ofdm_demod_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_OFDM_DEMOD_IMPL_H */
