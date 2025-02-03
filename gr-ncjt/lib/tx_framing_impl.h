/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_TX_FRAMING_IMPL_H
#define INCLUDED_NCJT_TX_FRAMING_IMPL_H

#include <gnuradio/ncjt/tx_framing.h>

namespace gr::ncjt
{

class tx_framing_impl : public tx_framing
{
private:
    const int SYM_LEN = 80; // OFDM symbol length

    int d_nstrm;          // number of streams
    int d_frame_length;   // full transmission frame length (beacon, data, and padding)
    int d_beacon_len;     // beacon length in samples
    int d_data_length;    // data symbols length in samples
    int d_padding_length; // zero padding length

    double d_repeat_interval;
    uint64_t d_time_secs;
    double d_time_fracs;

    gr_complex *d_beacon_data;
    const bool d_debug;
    pmt::pmt_t _id;

    uint64_t
    read_beacon_data(const char *filename);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    tx_framing_impl(int nstrm, int ndatasyms, const char *filename, double fs,
                    double interval, int starttime, int padding, bool debug);
    ~tx_framing_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRAMING_IMPL_H */
