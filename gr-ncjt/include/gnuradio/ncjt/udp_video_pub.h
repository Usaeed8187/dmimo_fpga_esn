/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UDP_VIDEO_PUB_H
#define INCLUDED_NCJT_UDP_VIDEO_PUB_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API udp_video_pub : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<udp_video_pub> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::udp_video_pub.
   *
   * To avoid accidental use of raw pointers, ncjt::udp_video_pub's
   * constructor is in a private implementation
   * class. ncjt::udp_video_pub::make is the public interface for
   * creating new instances.
   */
  static sptr make(const std::string &grp, int port);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_UDP_VIDEO_PUB_H */
