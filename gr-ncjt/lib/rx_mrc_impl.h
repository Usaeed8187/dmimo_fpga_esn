/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RX_MRC_IMPL_H
#define INCLUDED_NCJT_RX_MRC_IMPL_H

#include <gnuradio/ncjt/rx_mrc.h>

namespace gr::ncjt
{

class rx_mrc_impl : public rx_mrc
{
private:
    int d_scnum; // number of valid subcarriers
    int d_sdnum; // number data subcarriers
    int d_nrx;  // number of receivers
    int d_num_symbols;  // number of OFDM data symbols
    int d_cur_symbol; // current OFDM symbol
    int d_total_pkts; // total number of packets received

    gr_complex *d_chan_est; // channel estimation
    float d_chan_scaling[8]; // average channel power
    bool d_chan_est_ready;

    const bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    add_packet_tag(uint64_t offset, int packet_len);

public:
    rx_mrc_impl(int fft_size, int nrx, int nsymbols, bool debug);
    ~rx_mrc_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RX_MRC_IMPL_H */
