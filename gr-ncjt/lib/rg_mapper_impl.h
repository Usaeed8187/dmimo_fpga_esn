/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_RG_MAPPER_IMPL_H
#define INCLUDED_NCJT_RG_MAPPER_IMPL_H

#include <gnuradio/ncjt/rg_mapper.h>

namespace gr::ncjt
{

class rg_mapper_impl : public rg_mapper
{
private:
    const int SC_NUM = 56; // valid number of subcarriers
    const int SD_NUM = 52; // number of data subcarriers
    int d_nstrm; // number of streams
    int d_framelen; // input data frame length in samples
    int d_modtype; // modulation type (2-QPSK, 4-16QAM, 6-64QAM)
    int d_ndatasyms; // total number of OFDM symbols)
    int d_nsyms_per_stream; // QAM symbols per stream
    int d_total_symbols_required; // total number of QAM symbols for each data frame

    int d_fftsize; // FFT size
    int d_npt; // number of pilots
    gr_complex d_phaseshift[4][56];

    unsigned d_pilot_lsfr; // LSFR state for pilot parity sequence
    float d_cur_pilot[8][8]; // pilots for current OFDM symbol (mode 4 or 8)

    static const std::vector<gr_complex> CONST_QPSK; // QPSK constellation
    static const std::vector<gr_complex> CONST_16QAM; // 16QAM constellation
    static const std::vector<gr_complex> CONST_64QAM; // 64QAM constellation

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    update_pilots(int symidx);

public:
    rg_mapper_impl(int nstrm, int framelen, int ndatasyms, int npilotsyms, int modtype, bool debug);
    ~rg_mapper_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RG_MAPPER_IMPL_H */
