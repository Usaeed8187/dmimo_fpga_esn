/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_RG_DEMAPPER_H
#define INCLUDED_NCJT_RG_DEMAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief The \p rg_demapper block reassembles subcarriers from multiple streams (optionally with CSI) back into a single stream of data symbols and associated CSI.
     *
     * \details
     *  - If \p ctrl is \c true, the first 64 QPSK symbols on each stream are treated as control symbols. These are demodulated internally to extract parameters such as sequence number, modulation type, etc.
     *  - After that, the block interleaves the data symbols from each stream into a single output stream.
     *  - If \p usecsi is \c true, the block will also read CSI from the additional input ports and pass that along on the second output port. Otherwise, the second output port is set to unity CSI.
     *  - The block attaches tags with information from the control symbols (e.g., \c rx_ctrl_ok, \c rx_seqno, \c rx_data_checksum) to the output stream for downstream blocks.
     *
     * **Inputs/Outputs**
     *  - **Input Ports**:
     *    - If \p usecsi == \c false, there are \p nstrm input ports, each with \c (sc_num - pilot_count)×n_ofdm_syms data symbols.
     *    - If \p usecsi == \c true, there are 2×\p nstrm input ports: the first \p nstrm for data symbols, the second \p nstrm for CSI.
     *    - If \p ctrl == \c true, the first 64 symbols on each \p data input port are used for control.
     *  - **Output Ports** (2 ports):
     *    - **Port 0** : A single interleaved stream of all data symbols from all \p nstrm ports (minus 64 control symbols if \p ctrl == \c true).
     *    - **Port 1** : The corresponding CSI for each data symbol, either from the input CSI ports or set to \c 1.0 if \p usecsi == \c false.
     *
     * **Usage Example**
     * Often used right before the \p demapper block to turn these interleaved symbols plus CSI into demapped bits.
     */
    class NCJT_API rg_demapper : virtual public gr::tagged_stream_block
    {
    public:
      typedef std::shared_ptr<rg_demapper> sptr;

      /*!
       * \brief Creates a new instance of ncjt::rg_demapper.
       */
      static sptr make(int phase,
                       int rgmode,
                       int nstrm,
                       bool usecsi,
                       bool tag_snr,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_RG_DEMAPPER_H */
