/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_MAPPER_MUXER_H
#define INCLUDED_NCJT_MAPPER_MUXER_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief The \p mapper_muxer block maps input bits into QAM symbols with control always enabled.
     *
     * \details
     *  - To be completed with detailed description of the block's functionality.
     * **Input/Output**
     *  - **Input** : A stream of \c uint8_t bits.
     *  - **Output** : A stream of \c gr_complex QAM symbols.
     */
    class NCJT_API mapper_muxer : virtual public gr::block
    {
    public:
      typedef std::shared_ptr<mapper_muxer> sptr;

      /*!
       * \brief Creates a new instance of ncjt::mapper_muxer.
       */
      static sptr make(int rgmode,
                       int nstrm,
                       int phase1_modtype,
                       int phase2_modtype,
                       int phase3_modtype,
                       bool use_polar,
                       int code_rate,
                       bool deterministic_input,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_MAPPER_MUXER_H */
