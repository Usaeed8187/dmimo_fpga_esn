/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_OFDM_DEMOD_H
#define INCLUDED_NCJT_OFDM_DEMOD_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API ofdm_demod : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<ofdm_demod> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::ofdm_demod.
     *
     * To avoid accidental use of raw pointers, ncjt::ofdm_demod's
     * constructor is in a private implementation
     * class. ncjt::ofdm_demod::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int cplen, int symoffset = 12);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_OFDM_DEMOD_H */
