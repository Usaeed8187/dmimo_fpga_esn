/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_ADD_PREAMBLE_H
#define INCLUDED_NCJT_ADD_PREAMBLE_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API add_preamble : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<add_preamble> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::add_preamble.
     *
     * To avoid accidental use of raw pointers, ncjt::add_preamble's
     * constructor is in a private implementation
     * class. ncjt::add_preamble::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, const char *filename, int delay, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_ADD_PREAMBLE_H */
