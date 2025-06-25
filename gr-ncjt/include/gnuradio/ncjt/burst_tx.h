/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_BURST_TX_H
#define INCLUDED_NCJT_BURST_TX_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API burst_tx : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<burst_tx> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::burst_tx.
     *
     * To avoid accidental use of raw pointers, ncjt::burst_tx's
     * constructor is in a private implementation
     * class. ncjt::burst_tx::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int ntx, const char *filename, double samplerate, int pktspersec,
         int pktsize, double starttime, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_BURST_TX_H */
