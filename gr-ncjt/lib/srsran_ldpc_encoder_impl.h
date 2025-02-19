/* -*- c++ -*- */
/*
 * Copyright 2025 Yuanzhi Zhang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SRSRAN_LDPC_ENCODER_IMPL_H
#define INCLUDED_NCJT_SRSRAN_LDPC_ENCODER_IMPL_H

#include <gnuradio/ncjt/srsran_ldpc_encoder.h>
#include <srsran/channel_coding_factories.h>
#include "srsran/adt/bit_buffer.h"
#include "srsran/adt/span.h"
#include "srsran/srsvec/bit.h"
#include <vector>

using namespace srsran;
namespace gr {
  namespace ncjt {

    class srsran_ldpc_encoder_impl : public srsran_ldpc_encoder
    {
     private:
      int d_message_length; // in bit
      int d_padded_message_length; // in bit
      int d_codeword_length; // in bit
      int d_lifting_size;
      bool d_is_input_byte;
      bool d_debug;

      codeblock_metadata d_cfg;

      std::unique_ptr<ldpc_encoder> d_encoder;
      std::unique_ptr<ldpc_rate_matcher> d_matcher;

      dynamic_bit_buffer d_padded_message_packed;
      dynamic_bit_buffer d_rm_buffer;
      std::vector<uint8_t> d_unpacked;

      int d_input_stream_len;

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      srsran_ldpc_encoder_impl(std::string mod_order, int message_length, int codeword_length, bool is_input_byte, bool debug);
      ~srsran_ldpc_encoder_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_SRSRAN_LDPC_ENCODER_IMPL_H */
