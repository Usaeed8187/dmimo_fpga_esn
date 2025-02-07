/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_MIMO_DETECT_IMPL_H
#define INCLUDED_NCJT_MIMO_DETECT_IMPL_H

#include <gnuradio/ncjt/mimo_detect.h>

namespace gr::ncjt
{

class mimo_detect_impl : public mimo_detect
{
private:
    int d_nss; // number of spatial streams
    int d_nrx; // number of receiver antennas
    int d_num_symbols; // number of symbols per packet
    int d_cur_symbol; // current OFDM symbol
    int d_total_pkts; // total number of packets received

    int d_scnum; // number of valid subcarriers
    int d_scdata; // number data subcarriers

    gr_complex *d_chan_est_buf; // channel estimation using H-LTFs
    gr_complex *d_mmse_coef; // MMSE channel estimation
    bool d_chan_est_ready;

    const int d_logfreq;
    const bool d_debug;

    void
    update_mmse_coef_2rx(float nvar);

    void
    update_mmse_coef_4rx(float nvar);

    void
    update_mmse_coef_nrx(float nvar);

    void
    add_packet_tag(uint64_t offset, int packet_len);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    mimo_detect_impl(int fftsize, int nss, int ndatasymbols, int logfreq, bool debug);
    ~mimo_detect_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MIMO_DETECT_IMPL_H */
