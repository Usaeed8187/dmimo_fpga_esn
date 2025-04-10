/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_PKT_ERR_H
#define INCLUDED_NCJT_PKT_ERR_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API pkt_err : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<pkt_err> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::pkt_err.
     *
     * To avoid accidental use of raw pointers, ncjt::pkt_err's
     * constructor is in a private implementation
     * class. ncjt::pkt_err::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrms, const char *filename, bool shortterm, int avgwindow, int logfreq = 10,
         bool logpkterr = false, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_PKT_ERR_H */
