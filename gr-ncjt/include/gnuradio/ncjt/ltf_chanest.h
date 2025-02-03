/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_LTF_CHANEST_H
#define INCLUDED_NCJT_LTF_CHANEST_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API ltf_chanest : virtual public gr::tagged_stream_block {
public:
  typedef std::shared_ptr<ltf_chanest> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::ltf_chanest.
     *
     * To avoid accidental use of raw pointers, ncjt::ltf_chanest's
     * constructor is in a private implementation
     * class. ncjt::ltf_chanest::make is the public interface for
     * creating new instances.
     */
  static sptr make(int fftsize, int ntx, int nrx, int npreamblesyms,
                   int ndatasyms, bool docsi, int logfreq = 10,
                   bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_LTF_CHANEST_H */
