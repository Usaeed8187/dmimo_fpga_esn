/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SIC_DETECT_H
#define INCLUDED_NCJT_SIC_DETECT_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API sic_detect : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<sic_detect> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::sic_detect.
     *
     * To avoid accidental use of raw pointers, ncjt::sic_detect's
     * constructor is in a private implementation
     * class. ncjt::sic_detect::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int nrx, int nss, int modtype, int ndatasymbols, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SIC_DETECT_H */
