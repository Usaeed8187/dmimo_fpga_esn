/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_RG_MAPPER_H
#define INCLUDED_NCJT_RG_MAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief The \p rg_mapper block arranges incoming QAM symbols into an OFDM resource grid, inserting pilot carriers where needed.
     *
     * \details
     * This is a \c tagged_stream_block that:
     *  - Receives a chunk of QAM symbols corresponding to one entire frame (usually from \p mapper_muxer).
     *  - Distributes those symbols into the resource grid of size \p sc_num x \p n_ofdm_syms for each \p nstrm output port.
     *  - Inserts pilot symbols in the specified \p pilot_sc_ind subcarriers.
     *  - Optionally applies a per-stream cyclic shift if \p addcs is \c true (often used in multi-antenna transmissions).
     *
     * **Parameters**
     *  - \p nstrm : Number of spatial streams (and number of output ports).
     *  - \p n_ofdm_syms : Number of OFDM symbols in the frame.
     *  - \p sc_num : Total subcarriers (e.g., 56).
     *  - \p pilot_sc_ind : Indices of pilot subcarriers within those 56 (e.g., [7,21,34,48]).
     *  - \p addcs : If \c true, apply additional cyclic shifts to each stream.
     *  - \p debug : If \c true, prints verbose debug info.
     *
     * **Input/Output**
     *  - **Input** (1 port): \n
     *    A tagged stream of \c gr_complex QAM symbols (one full OFDM frame).
     *  - **Output** (\p nstrm ports): \n
     *    Each port emits \c sc_num x n_ofdm_syms \c gr_complex values (the “resource grid”) with pilot subcarriers inserted. Port \c i corresponds to stream \c i.
     *
     * **Usage Example**
     * Typically, this block is followed by something that sends these resource grids to hardware or to a simulated channel (e.g., the \p noair block).
     */
    class NCJT_API rg_mapper : virtual public gr::tagged_stream_block
    {
    public:
      typedef std::shared_ptr<rg_mapper> sptr;

      /*!
       * \brief Creates a new instance of ncjt::rg_mapper.
       */
      static sptr make(int nstrm,
        int n_ofdm_syms,
        int sc_num,
        const std::vector<int> &pilot_sc_ind,
        bool addcs,
        bool debug, 
        int numue,
        int ueidx, 
        bool mucpt, 
        bool addltf,
        const char *ltfdata);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_RG_MAPPER_H */
