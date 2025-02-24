/* -*- c++ -*- */
/*
 * Copyright 2025 usama.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UL_PRECODING_OFFLINE_H
#define INCLUDED_NCJT_UL_PRECODING_OFFLINE_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API ul_precoding_offline : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<ul_precoding_offline> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::ul_precoding_offline.
   *
   * To avoid accidental use of raw pointers, ncjt::ul_precoding_offline's
   * constructor is in a private implementation
   * class. ncjt::ul_precoding_offline::make is the public interface for
   * creating new instances.
   */
  static sptr make(int nss, int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms,
                   int numprecodedsyms, bool eigenmode, const char *filename,
                   bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_UL_PRECODING_OFFLINE_H */
