/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SOFT_DEMAPPER_H
#define INCLUDED_NCJT_SOFT_DEMAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API soft_demapper : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<soft_demapper> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::soft_demapper.
     *
     * To avoid accidental use of raw pointers, ncjt::soft_demapper's
     * constructor is in a private implementation
     * class. ncjt::soft_demapper::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, int framelen, int modtype, bool usecsi, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SOFT_DEMAPPER_H */
