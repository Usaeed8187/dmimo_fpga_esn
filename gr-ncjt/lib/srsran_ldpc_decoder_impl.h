/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_SRSRAN_LDPC_DECODER_IMPL_H
#define INCLUDED_NCJT_SRSRAN_LDPC_DECODER_IMPL_H

#include <gnuradio/ncjt/srsran_ldpc_decoder.h>
#include <srsran/channel_coding_factories.h>
#include <srsran/ldpc/ldpc_decoder.h>
#include "srsran/adt/bit_buffer.h"
#include "srsran/adt/span.h"
#include "srsran/srsvec/bit.h"
#include <vector>

using namespace srsran;

namespace gr::ncjt
{

class srsran_ldpc_decoder_impl : public srsran_ldpc_decoder
{
private:
    const int d_max_blocks = 16; // maximum number of LDPC blocks to process
    int d_message_length;
    int d_padded_message_length;
    int d_codeword_length;
    int d_lifting_size;
    bool d_is_output_byte;
    bool d_debug;

    codeblock_metadata d_cfg;
    ldpc_decoder::configuration d_cfg_dec;

    std::unique_ptr<ldpc_decoder> d_decoder;
    std::unique_ptr<ldpc_rate_dematcher> d_dematcher;

    std::vector<log_likelihood_ratio> d_llrs;
    std::vector<log_likelihood_ratio> d_llrs_dematched;
    dynamic_bit_buffer d_decoded;
    std::vector<uint8_t> d_decoded_bits_unpacked;

    int d_output_stream_len;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    srsran_ldpc_decoder_impl(std::string mod_order,
                             int message_length,
                             int codeword_length,
                             bool is_output_byte,
                             bool debug);
    ~srsran_ldpc_decoder_impl();

    int
    work(
        int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items
    );
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SRSRAN_LDPC_DECODER_IMPL_H */
