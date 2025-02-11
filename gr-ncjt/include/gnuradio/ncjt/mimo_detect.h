/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_MIMO_DETECT_H
#define INCLUDED_NCJT_MIMO_DETECT_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API mimo_detect : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<mimo_detect> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::mimo_detect.
     *
     * To avoid accidental use of raw pointers, ncjt::mimo_detect's
     * constructor is in a private implementation
     * class. ncjt::mimo_detect::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int nrx, int nss, int ndatasymbols, int logfreq, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MIMO_DETECT_H */
