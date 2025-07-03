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
     *  - Always inserts 64 control symbols per stream at the frame start.
     *  - Then, remaining bits are mapped into the chosen QAM constellation.
     *
     * **Parameters**
     *  - \p nstrm : Number of spatial streams.
     *  - \p modtype : Number of bits per symbol (2=QPSK, 4=16QAM, 6=64QAM, 8=256QAM).
     *  - \p n_ofdm_syms : Number of OFDM symbols per frame.
     *  - \p sd_num : Number of data subcarriers (e.g., 52).
     *  - \p use_polar : If \c true, uses polar encoding for control bits, otherwise uses repetition.
     *  - \p code_rate : Code rate (0: Disable, 1: 1/4, ..., 7: 5/6).
     *  - \p debug : If \c true, prints verbose debug info.
     *
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
      static sptr make(int nstrm,
                       int phase1_modtype,
                       int phase2_modtype,
                       int phase3_modtype,
                       int n_ofdm_syms,
                       int sd_num,
                       bool use_polar,
                       int code_rate,
                       bool deterministic_input,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_MAPPER_MUXER_H */
