/* -*- c++ -*- */
/*
 * Copyright 2025.
 *
 * A simple pass-through “no-air” block that can optionally remove
 * pilot subcarriers so that the demapper sees 52-subcarrier frames
 * (instead of 56). Now extended to add AWGN based on an SNR (dB) parameter.
 */

#ifndef INCLUDED_NCJT_NOAIR_H
#define INCLUDED_NCJT_NOAIR_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>
#include <vector>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief The noair block acts as an ideal "over-the-air" emulator in the absence of real USRPs.
     */
    class NCJT_API noair : virtual public gr::tagged_stream_block
    {
    public:
      typedef std::shared_ptr<noair> sptr;

      /*!
       * \brief Creates a new instance of ncjt::noair.
       *  - Removes pilot subcarriers from each OFDM symbol so that only data subcarriers remain.
       *  - Generates a dummy CSI output (all set to 1.0+0.0j).
       *  - Optionally adds AWGN based on the specified \p snr_db.
       *  - Throttles the packet rate if \p frame_per_sec > 0 by enforcing a delay between frames.
       *
       * **Inputs:**
       *  - **Input (1 port):** Tagged stream of \c gr_complex representing the entire OFDM frame (i.e., \c (sc_num * n_ofdm_syms) samples).
       *
       * **Outputs:**
       *  - **Output0:** \c (sc_num - len(pilot_sc_ind)) * n_ofdm_syms complex samples (data subcarriers, possibly with noise).
       *  - **Output1:** \c (sc_num - len(pilot_sc_ind)) * n_ofdm_syms complex samples of CSI (all set to \c 1.0+0.0j).
       */
      static sptr make(int phase,
                       int rgmode,
                       float frame_per_sec,
                       float snr_db,
                       int num_drop_init_packets,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_NOAIR_H */
