/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_BURST_TX_IMPL_H
#define INCLUDED_NCJT_BURST_TX_IMPL_H

#include <gnuradio/ncjt/burst_tx.h>

namespace gr::ncjt
{

class burst_tx_impl : public burst_tx
{
private:
    double d_repeat_interval;
    uint64_t d_time_secs;
    double d_time_fracs;
    int d_pktsize;      // packet size
    int d_data_len;     // total length of data samples
    int d_buf_pos;      // current position of the data buffer
    bool d_first_burst;

    gr_complex *d_frame_data;
    pmt::pmt_t _id;
    const bool d_debug;

    uint64_t
    read_frame_data(const char *filename);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    burst_tx_impl(const char *filename, double samplerate, int pktspersec,
                  int pktsize, double starttime, bool debug);
    ~burst_tx_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_BURST_TX_IMPL_H */
