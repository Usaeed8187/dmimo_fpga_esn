/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_VIDEO_SOURCE_H
#define INCLUDED_NCJT_VIDEO_SOURCE_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API video_source : virtual public gr::block
{
public:
    typedef std::shared_ptr<video_source> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::video_source.
     *
     * To avoid accidental use of raw pointers, ncjt::video_source's
     * constructor is in a private implementation
     * class. ncjt::video_source::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int framelen, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_VIDEO_SOURCE_H */
