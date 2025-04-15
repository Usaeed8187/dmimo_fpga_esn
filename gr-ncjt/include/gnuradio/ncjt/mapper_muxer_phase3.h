/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_MAPPER_MUXER_PHASE3_H
#define INCLUDED_NCJT_MAPPER_MUXER_PHASE3_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API mapper_muxer_phase3 : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<mapper_muxer_phase3> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of
   * ncjt::mapper_muxer_phase3.
   *
   * To avoid accidental use of raw pointers, ncjt::mapper_muxer_phase3's
   * constructor is in a private implementation
   * class. ncjt::mapper_muxer_phase3::make is the public interface for
   * creating new instances.
   */
  static sptr make(int nstrm, int n_ofdm_syms, int sd_num,
                   bool use_polar, bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_MAPPER_MUXER_PHASE3_H */
