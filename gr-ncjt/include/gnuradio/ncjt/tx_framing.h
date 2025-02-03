/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_TX_FRAMING_H
#define INCLUDED_NCJT_TX_FRAMING_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API tx_framing : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<tx_framing> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::tx_framing.
     *
     * To avoid accidental use of raw pointers, ncjt::tx_framing's
     * constructor is in a private implementation
     * class. ncjt::tx_framing::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, int ndatasyms, const char *filename, double fs,
         double interval, int starttime, int padding, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRAMING_H */
