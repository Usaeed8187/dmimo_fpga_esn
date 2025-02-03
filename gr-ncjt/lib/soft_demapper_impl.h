/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SOFT_DEMAPPER_IMPL_H
#define INCLUDED_NCJT_SOFT_DEMAPPER_IMPL_H

#include <gnuradio/ncjt/soft_demapper.h>

namespace gr::ncjt
{

class soft_demapper_impl : public soft_demapper
{
private:
    const int SC_NUM = 56; // number of valid subcarriers
    const int SD_NUM = 52; // number of data subcarriers
    int d_nstrm; // number of streams
    int d_framelen; // data frame length in samples
    int d_modtype;   // modulation type (number of bits per symbol)
    bool d_usecsi;   // using CSI for QAM symbols

    pmt::pmt_t _id;
    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    add_packet_tag(uint64_t offset, int packet_len);

public:
    soft_demapper_impl(int nstrm, int framelen, int modtype, bool usecsi, bool debug);
    ~soft_demapper_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SOFT_DEMAPPER_IMPL_H */
