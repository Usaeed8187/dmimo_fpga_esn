/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_SKIP_DATA_H
#define INCLUDED_NCJT_SKIP_DATA_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API skip_data : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<skip_data> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::skip_data.
     *
     * To avoid accidental use of raw pointers, ncjt::skip_data's
     * constructor is in a private implementation
     * class. ncjt::skip_data::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, int skiplen, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SKIP_DATA_H */
