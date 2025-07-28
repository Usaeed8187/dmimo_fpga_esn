/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_P1_PRECODING_H
#define INCLUDED_NCJT_P1_PRECODING_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API p1_precoding : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<p1_precoding> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::p1_precoding.
   *
   * To avoid accidental use of raw pointers, ncjt::p1_precoding's
   * constructor is in a private implementation
   * class. ncjt::p1_precoding::make is the public interface for
   * creating new instances.
   */
  static sptr make(int num_ue, int nss, int ntx_gnb, int num_iter,
                   int numltfsyms, int numdatasyms, bool wideband, int nfft,
                   int num_sc, int logfreq, bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_P1_PRECODING_H */
