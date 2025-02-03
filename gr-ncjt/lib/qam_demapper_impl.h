/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_QAM_DEMAPPER_IMPL_H
#define INCLUDED_NCJT_QAM_DEMAPPER_IMPL_H

#include <gnuradio/ncjt/qam_demapper.h>

namespace gr::ncjt
{

class qam_demapper_impl : public qam_demapper
{
private:
    int d_num_strms; // number of spatial streams
    int d_modtype;   // modulation type (number of bits per symbol)
    bool d_usecsi;   // using CSI for QAM symbols
    const bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    qam_demapper_impl(int nstrms, int modtype, bool usecsi, bool debug);
    ~qam_demapper_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_QAM_DEMAPPER_IMPL_H */
