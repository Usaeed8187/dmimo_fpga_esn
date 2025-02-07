/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_SYNC_ALL_H
#define INCLUDED_NCJT_SYNC_ALL_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API sync_all : virtual public gr::block
{
public:
    typedef std::shared_ptr<sync_all> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::sync_all.
     *
     * To avoid accidental use of raw pointers, ncjt::sync_all's
     * constructor is in a private implementation
     * class. ncjt::sync_all::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int num_rx, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SYNC_ALL_H */
