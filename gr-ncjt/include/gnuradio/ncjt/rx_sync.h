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
    make(int nchans, int npreamblesyms, int ndatasyms, double sampling_freq, int pktspersec,
         double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RX_SYNC_H */
