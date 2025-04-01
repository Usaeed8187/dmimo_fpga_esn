/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_CHECK_CTRLDATA_H
#define INCLUDED_NCJT_CHECK_CTRLDATA_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API check_ctrldata : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<check_ctrldata> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of ncjt::check_ctrldata.
   *
   * To avoid accidental use of raw pointers, ncjt::check_ctrldata's
   * constructor is in a private implementation
   * class. ncjt::check_ctrldata::make is the public interface for
   * creating new instances.
   */
  static sptr make(int nstrm, int ctrl_datalen, bool has_txseq, bool has_rxseq, bool has_llr,
                   bool extract_llr, bool waitrdy, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_CHECK_CTRLDATA_H */
