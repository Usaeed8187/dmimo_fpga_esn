/* -*- c++ -*- */
/*
 * Copyright 2024 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_PKT_DETECT_H
#define INCLUDED_NCJT_PKT_DETECT_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API pkt_detect : virtual public gr::block
{
public:
    typedef std::shared_ptr<pkt_detect> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::pkt_detect.
     *
     * To avoid accidental use of raw pointers, ncjt::pkt_detect's
     * constructor is in a private implementation
     * class. ncjt::pkt_detect::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nchans, int preamblelen, int dataframelen,
         double samplerate, int pktspersec, double acorr_thrd,
         double xcorr_thrd, int max_corr_len, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_PKT_DETECT_H */
