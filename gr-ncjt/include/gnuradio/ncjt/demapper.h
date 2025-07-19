/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless@VT.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_DEMAPPER_H
#define INCLUDED_NCJT_DEMAPPER_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr
{
  namespace ncjt
  {

    /*!
     * \brief The \p demapper block takes an interleaved stream of complex data symbols plus CSI and produces hard-decision bits.
     *
     * \details
     *  - The first input port receives the data symbols (already interleaved from multiple streams).
     *  - The second input port receives CSI values (or 1.0 if no real CSI is available).
     *  - Once it has enough symbols for a “packet” (as indicated by the \c packet_len and other tags like \c rx_modtype), it demaps them using the indicated modulation.
     *  - It optionally verifies a CRC if a checksum was provided in the \c rx_data_checksum tag, attaching a \c rx_data_crc_passed tag to the output indicating success/failure.
     *  - The output is a stream of \c uint8_t bits.
     *
     */
    class NCJT_API demapper : virtual public gr::block
    {
    public:
      typedef std::shared_ptr<demapper> sptr;

      /*!
       * \brief Creates a new instance of ncjt::demapper.
       *
       * \param debug Whether to enable verbose debugging
       */
      static sptr make(int rgmode, 
                       bool coded,
                       bool deterministic_input,
                       bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_DEMAPPER_H */
