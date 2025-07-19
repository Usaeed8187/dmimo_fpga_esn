/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_PDC_H
#define INCLUDED_NCJT_PDC_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief <+description of block+>
     * \ingroup ncjt
     *
     */
    class NCJT_API pdc : virtual public gr::block
    {
    public:
      typedef std::shared_ptr<pdc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ncjt::pdc.
       *
       * To avoid accidental use of raw pointers, ncjt::pdc's
       * constructor is in a private implementation
       * class. ncjt::pdc::make is the public interface for
       * creating new instances.
       */
      static sptr make(int rgmode,
                       bool majority_enabled,
                       int num_copies,
                       int expire_ms,
                       int num_threads,
                       bool deterministic_input,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_PDC_H */
