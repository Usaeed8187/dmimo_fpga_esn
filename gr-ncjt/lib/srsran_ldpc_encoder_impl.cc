/* -*- c++ -*- */
/*
 * Copyright 2025 Yuanzhi Zhang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "srsran_ldpc_encoder_impl.h"
#include "srsran/ldpc/ldpc_encoder_buffer.h"
#include <iomanip>

namespace gr {
  namespace ncjt {

    using input_type = char;
    using output_type = char;
    srsran_ldpc_encoder::sptr
    srsran_ldpc_encoder::make(std::string mod_order, int message_length, int codeword_length, bool is_input_byte, bool debug)
    {
      return gnuradio::make_block_sptr<srsran_ldpc_encoder_impl>(
        mod_order, message_length, codeword_length, is_input_byte, debug);
    }


    /*
     * The private constructor
     */
    srsran_ldpc_encoder_impl::srsran_ldpc_encoder_impl(std::string mod_order, int message_length, int codeword_length, bool is_input_byte, bool debug)
      : gr::tagged_stream_block("srsran_ldpc_encoder",gr::io_signature::make(1, 1 , sizeof(input_type)),
      gr::io_signature::make(1, 1, sizeof(output_type)), "packet_len"),
      d_message_length(message_length), d_codeword_length(codeword_length), d_is_input_byte(is_input_byte), d_debug(debug)
    {
      std::shared_ptr<ldpc_encoder_factory> enc_factory = create_ldpc_encoder_factory_sw("avx2");
      std::shared_ptr<ldpc_rate_matcher_factory> matcher_factory = create_ldpc_rate_matcher_factory_sw();
      if (!enc_factory || !matcher_factory)
        std::cerr << "Failed to create encoder or matcher factory!" << std::endl;
      d_encoder = enc_factory->create();
      d_matcher = matcher_factory->create();

      if (!d_encoder || !d_matcher)
        std::cerr << "Failed to create encoder or matcher instance!" << std::endl;

      // Check if the message and codeword lengths are valid
      if(d_message_length <= 0 || d_codeword_length <= 0 || d_message_length >= d_codeword_length)
        throw std::invalid_argument("Invalid message or codeword length");

      // input tagged stream length
      d_input_stream_len = d_is_input_byte ? d_message_length/8 : d_message_length; // assume message_length is multiple of 8 if is_input_byte

      // Set the codeblock metadata configuration
      d_lifting_size = ldpc::compute_lifting_size_BG1(units::bits(message_length));
      d_cfg.tb_common = {ldpc_base_graph_type::BG1, static_cast<ldpc::lifting_size_t>(d_lifting_size)};
      d_cfg.tb_common.mod = modulation_scheme_from_string(mod_order);
      d_padded_message_length = d_lifting_size*22;
      d_cfg.cb_specific.nof_filler_bits = d_padded_message_length - d_message_length;

      // setup buffer
      d_padded_message_packed.resize(d_padded_message_length);
      d_rm_buffer.resize(d_codeword_length);
      d_unpacked.resize(d_codeword_length);

      set_tag_propagation_policy(block::TPP_DONT);

      std::cout << "----------------------------------------------------------------------------------------------------------------"<<std::endl;
      std::cout << "| Message Len   | Codeword Len  | Lifting Size  | Padded Len    | Code Rate     |"<<std::endl;
      std::cout << "----------------------------------------------------------------------------------------------------------------"<<std::endl;


      std::cout << "|" << std::setw(20) << d_message_length << "|" << std::setw(20) << d_codeword_length << "|"
            << std::setw(20) << d_lifting_size << "|" << std::setw(20) << d_cfg.cb_specific.nof_filler_bits << "|"
            << std::setw(20) << d_message_length/float(d_codeword_length) << "|"<< std::endl;
      std::cout << "----------------------------------------------------------------------------------------------------------------"<<std::endl;

    }

    /*
     * Our virtual destructor.
     */
    srsran_ldpc_encoder_impl::~srsran_ldpc_encoder_impl()
    {
    }

    int
    srsran_ldpc_encoder_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      return d_codeword_length;
    }

    int
    srsran_ldpc_encoder_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      if(ninput_items[0] != d_input_stream_len) {
        std::cerr << "Got wrong message length " << ninput_items[0] <<
        ", expect "<< d_input_stream_len << std::endl;
        return 0;
      }
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);

      std::vector<uint8_t> message(in, in + d_input_stream_len);
      // padding bits
      if(d_is_input_byte)    // byte case
      {
        for(int i = 0; i < (d_cfg.cb_specific.nof_filler_bits+7)/8 ; ++i) message.push_back(0);
        srsvec::copy_offset(d_padded_message_packed, message, 0);
      }
      else                    // bit case
      {
        for(int i = 0; i < d_cfg.cb_specific.nof_filler_bits; ++i) message.push_back(0);
        srsvec::bit_pack(d_padded_message_packed, message);
      }


      const ldpc_encoder_buffer& encode_buffer = d_encoder->encode(d_padded_message_packed , d_cfg.tb_common);
      d_matcher->rate_match(d_rm_buffer, encode_buffer, d_cfg);

      // Unpack the encoded message
      srsvec::bit_unpack(d_unpacked, d_rm_buffer);
      std::copy(d_unpacked.begin(), d_unpacked.end(), out);

      add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("packet_len"),
                    pmt::from_long(d_codeword_length),
                    pmt::string_to_symbol(name()));

      return d_codeword_length;
    }

  } /* namespace ncjt */
} /* namespace gr */
