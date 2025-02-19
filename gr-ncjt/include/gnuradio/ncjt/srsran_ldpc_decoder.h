/* -*- c++ -*- */
/*
 * Copyright 2025 Yuanzhi Zhang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SRSRAN_LDPC_DECODER_H
#define INCLUDED_NCJT_SRSRAN_LDPC_DECODER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace ncjt {

    /*!
     * \brief <+description of block+>
     * \ingroup ncjt
     *
     */
    class NCJT_API srsran_ldpc_decoder : virtual public gr::tagged_stream_block
    {
     public:
      typedef std::shared_ptr<srsran_ldpc_decoder> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ncjt::srsran_ldpc_decoder.
       *
       * To avoid accidental use of raw pointers, ncjt::srsran_ldpc_decoder's
       * constructor is in a private implementation
       * class. ncjt::srsran_ldpc_decoder::make is the public interface for
       * creating new instances.
       */
      static sptr make(std::string mod_order, int message_length, int codeword_length, bool is_output_byte, bool debug);
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_SRSRAN_LDPC_DECODER_H */
