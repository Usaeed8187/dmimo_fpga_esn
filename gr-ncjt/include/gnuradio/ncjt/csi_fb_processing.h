/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_CSI_FB_PROCESSING_H
#define INCLUDED_NCJT_CSI_FB_PROCESSING_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
namespace ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API csi_fb_processing : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<csi_fb_processing> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::csi_fb_processing.
   *
   * To avoid accidental use of raw pointers, ncjt::csi_fb_processing's
   * constructor is in a private implementation
   * class. ncjt::csi_fb_processing::make is the public interface for
   * creating new instances.
   */
  static sptr make(int num_ue, int rx_modtype, int nss, int ntx_gnb,
                   bool wideband, int nrb, int pmi_bits, int cqi_bits,
                   int crc_bits, int total_csi_bits, float polar_code_rate,
                   bool debug);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_CSI_FB_PROCESSING_H */
