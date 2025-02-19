/* -*- c++ -*- */
/*
 * Copyright 2025 Yuanzhi Zhang.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "srsran_ldpc_decoder_impl.h"
#include <chrono>

# include "utils.h"

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::microseconds us;
typedef std::chrono::duration<float> fsec;

namespace gr {
  namespace ncjt {

    using input_type = float;
    using output_type = char;
    srsran_ldpc_decoder::sptr
    srsran_ldpc_decoder::make(std::string mod_order, int message_length, int codeword_length, bool is_output_byte, bool debug)
    {
      return gnuradio::make_block_sptr<srsran_ldpc_decoder_impl>(
        mod_order, message_length, codeword_length, is_output_byte, debug);
    }


    /*
     * The private constructor
     */
    srsran_ldpc_decoder_impl::srsran_ldpc_decoder_impl(std::string mod_order, int message_length, int codeword_length, bool is_output_byte, bool debug)
      : gr::tagged_stream_block("srsran_ldpc_decoder",
        gr::io_signature::make(1, 1, sizeof(input_type)),
        gr::io_signature::make(1, 1, sizeof(output_type)), "packet_len"),
        d_message_length(message_length), d_codeword_length(codeword_length), d_is_output_byte(is_output_byte), d_debug(debug)
    {
      std::shared_ptr<ldpc_decoder_factory> dec_factory = create_ldpc_decoder_factory_sw("avx2");
      std::shared_ptr<ldpc_rate_dematcher_factory> dematcher_factory = create_ldpc_rate_dematcher_factory_sw("avx2");
      if (!dec_factory || !dematcher_factory)
        std::cerr << "Failed to create decoder or dematcher factory!" << std::endl;
      d_decoder = dec_factory->create();
      d_dematcher = dematcher_factory->create();

      if (!d_decoder || !d_dematcher)
        std::cerr << "Failed to create encoder or decoder instance!" << std::endl;

      // Check if the message and codeword lengths are valid
      if(d_message_length <= 0 || d_codeword_length <= 0 || d_message_length >= d_codeword_length)
        throw std::invalid_argument("Invalid message or codeword length");

      // output tagged stream length
      d_output_stream_len = d_is_output_byte ? d_message_length/8 : d_message_length; // assume message_length is multiple of 8 if is_output_byte

      // Set the codeblock metadata configuration
      d_lifting_size = ldpc::compute_lifting_size_BG1(units::bits(message_length));
      d_cfg.tb_common = {ldpc_base_graph_type::BG1, static_cast<ldpc::lifting_size_t>(d_lifting_size)};
      d_cfg.tb_common.mod = modulation_scheme_from_string(mod_order);
      d_padded_message_length = d_lifting_size*22;
      d_cfg.cb_specific.nof_filler_bits = d_padded_message_length - message_length;
      d_cfg_dec = {{d_cfg.tb_common}, {}};

      d_llrs.resize(d_codeword_length);
      d_llrs_dematched.resize(d_lifting_size*66);
      d_decoded.resize(d_padded_message_length);
      d_decoded_bits_unpacked.resize(d_padded_message_length);

      set_tag_propagation_policy(block::TPP_DONT);
      dout << "Decoder Initialized!"<<std::endl;
      dout << "Message length" << message_length <<std::endl;
      dout << "Lifting size = "<< d_lifting_size <<std::endl;
    }

    /*
     * Our virtual destructor.
     */
    srsran_ldpc_decoder_impl::~srsran_ldpc_decoder_impl()
    {
    }

    int
    srsran_ldpc_decoder_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      return d_message_length;
    }

    int
    srsran_ldpc_decoder_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      if(ninput_items[0] != d_codeword_length) {
        std::cerr << "Got wrong codeword length" << ninput_items[0] <<
        ", expect "<< d_codeword_length << std::endl;
        return 0;
      }
      auto de_t0 = Time::now();
      // std::cout << "Decoder work"<<std::endl;
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);

      for (int i = 0; i < d_codeword_length; ++i) {
        d_llrs[i] = log_likelihood_ratio::quantize(in[i], 15); // Convert to LLRs
      }

      d_dematcher->rate_dematch(d_llrs_dematched, d_llrs, true, d_cfg);

      d_decoder->decode(d_decoded, d_llrs_dematched, nullptr, d_cfg_dec);

      if(d_is_output_byte)    // byte output case
      {
        std::copy(d_decoded.get_buffer().begin(), d_decoded.get_buffer().begin()+d_output_stream_len, out);
      }else                 // bit output case
      {
        srsvec::bit_unpack(d_decoded_bits_unpacked, d_decoded);
        std::copy(d_decoded_bits_unpacked.begin(), d_decoded_bits_unpacked.end() - d_cfg.cb_specific.nof_filler_bits, out);
      }

      add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("packet_len"),
                    pmt::from_long(d_output_stream_len),
                    pmt::string_to_symbol(name()));

      // std::cout << "Decoder work done"<<std::endl;
      auto de_t1 = Time::now();
      fsec de_fs = de_t1 - de_t0;
      us de_d = std::chrono::duration_cast<us>(de_fs);
      dout << "Decoding Speed:" << d_message_length / float(de_d.count()) * 0.1192 << "MB/s\n"; // 0.1192 = 1,000,000/(1024*1024*8);

      return d_output_stream_len;
    }

  } /* namespace ncjt */
} /* namespace gr */
