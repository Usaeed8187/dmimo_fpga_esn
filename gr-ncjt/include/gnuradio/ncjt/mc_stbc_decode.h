/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_MC_STBC_DECODE_H
#define INCLUDED_NCJT_MC_STBC_DECODE_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API mc_stbc_decode : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<mc_stbc_decode> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::mc_stbc_decode.
     *
     * To avoid accidental use of raw pointers, ncjt::mc_stbc_decode's
     * constructor is in a private implementation
     * class. ncjt::mc_stbc_decode::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int ndatasyms, int npilotsyms, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MC_STBC_DECODE_H */
