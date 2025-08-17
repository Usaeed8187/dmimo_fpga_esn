/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RX_MRC_H
#define INCLUDED_NCJT_RX_MRC_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API rx_mrc : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<rx_mrc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::rx_mrc.
     *
     * To avoid accidental use of raw pointers, ncjt::rx_mrc's
     * constructor is in a private implementation
     * class. ncjt::rx_mrc::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int rgmode, int nrx, int nsymbols, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RX_MRC_H */
