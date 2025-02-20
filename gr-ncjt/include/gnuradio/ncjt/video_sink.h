/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_VIDEO_SINK_H
#define INCLUDED_NCJT_VIDEO_SINK_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API video_sink : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<video_sink> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::video_sink.
     *
     * To avoid accidental use of raw pointers, ncjt::video_sink's
     * constructor is in a private implementation
     * class. ncjt::video_sink::make is the public interface for
     * creating new instances.
     */
  static sptr make(int framelen, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_VIDEO_SINK_H */
