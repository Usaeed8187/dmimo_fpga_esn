/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RX_SYNC_H
#define INCLUDED_NCJT_RX_SYNC_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API rx_sync : virtual public gr::block
{
public:
    typedef std::shared_ptr<rx_sync> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::rx_sync.
     *
     * To avoid accidental use of raw pointers, ncjt::rx_sync's
     * constructor is in a private implementation
     * class. ncjt::rx_sync::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nchans, bool p1rx, int npreamblesyms, int ndatasyms, double sampling_freq, int pktspersec,
         bool p2rx, int hwdelayp2, int p2preamblelen, int p2framelen, double p2_start,
         bool p3rx, int hwdelayp3, int p3preamblelen, int p3framelen, double p3_start, double rxpwr_thrd,
         double acorr_thrd, double xcorr_thrd, int max_corr_len, bool lltf2 = false, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RX_SYNC_H */
