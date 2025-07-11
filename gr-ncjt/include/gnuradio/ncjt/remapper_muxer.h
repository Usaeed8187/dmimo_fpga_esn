/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_remapper_muxer_H
#define INCLUDED_NCJT_remapper_muxer_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API remapper_muxer : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<remapper_muxer> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of
   * ncjt::remapper_muxer.
   *
   * To avoid accidental use of raw pointers, ncjt::remapper_muxer's
   * constructor is in a private implementation
   * class. ncjt::remapper_muxer::make is the public interface for
   * creating new instances.
   */
  static sptr make(int phase, int rgmode, int nstrm, bool reencode, bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_remapper_muxer_H */
