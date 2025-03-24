/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_GNB_SYNC_H
#define INCLUDED_NCJT_GNB_SYNC_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr
{
namespace ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API gnb_sync : virtual public gr::block
{
public:
    typedef std::shared_ptr <gnb_sync> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::gnb_sync.
     *
     * To avoid accidental use of raw pointers, ncjt::gnb_sync's
     * constructor is in a private implementation
     * class. ncjt::gnb_sync::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nchans, double samplerate, int pktspersec, int hwdelayp2, int hwdelayp3,
         int p2_htlen, int p2_datalen, int p3_htlen, int p3_datalen, double p2_start, double p3_start,
         double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_GNB_SYNC_H */
