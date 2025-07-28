/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_CSI_QUANTIZER_H
#define INCLUDED_NCJT_CSI_QUANTIZER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API csi_quantizer : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<csi_quantizer> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::csi_quantizer.
   *
   * To avoid accidental use of raw pointers, ncjt::csi_quantizer's
   * constructor is in a private implementation
   * class. ncjt::csi_quantizer::make is the public interface for
   * creating new instances.
   */
  static sptr make(int fftsize, int ntx, int nrx, int nss, int rbg_size,
                   int n_ofdm_syms_fb, int logfreq, bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_CSI_QUANTIZER_H */
