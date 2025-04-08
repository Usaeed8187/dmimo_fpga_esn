/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "llr_combine_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

using output_type = float;
llr_combine::sptr
llr_combine::make(int nstrm, int modtype1, int modtype2, int blocksize, bool llrsum, bool majority, bool debug)
{
    return gnuradio::make_block_sptr<llr_combine_impl>(nstrm, modtype1, modtype2, blocksize, llrsum, majority, debug);
}

llr_combine_impl::llr_combine_impl(int nstrm, int modtype1, int modtype2, int blocksize, bool llrsum, bool majority, bool debug)
    : gr::tagged_stream_block(
    "llr_combine",
    gr::io_signature::make(nstrm, nstrm, sizeof(uint8_t)),
    gr::io_signature::make2(1, 3, sizeof(float), sizeof(uint8_t)),
    "packet_len"), d_accurate_llr(llrsum), d_majority(majority), d_debug(debug)
{
    d_nstrm = nstrm;
    if (modtype1 != 2 && modtype1 != 4 && modtype1 != 6 && modtype1 != 8)
        throw std::runtime_error("unsupported modulation type");
    d_modtype1 = modtype1;
    if (modtype2 != 2 && modtype2 != 4 && modtype2 != 6 && modtype2 != 8)
        throw std::runtime_error("unsupported modulation type");
    d_modtype2 = modtype2;

    d_llr_data = malloc_float(sizeof(float) * SD_NUM * d_nstrm);

}

llr_combine_impl::~llr_combine_impl()
{
    if (d_llr_data != nullptr)
        volk_free(d_llr_data);
}

int
llr_combine_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    return (min_input_items - SD_NUM);
}

int
llr_combine_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    // Retrieve LLR data tag
    std::vector<gr::tag_t> d_tags;
    for (int ch = 0; ch < d_nstrm; ch++)
    {
        get_tags_in_window(d_tags, ch, 0, min_input_items,
                           pmt::string_to_symbol("llr"));
        if (d_tags.empty())
            throw std::runtime_error("ERROR: cannot find LLR data tag");

        pmt::pmt_t llr_blob = d_tags[0].value;
        int dlen = pmt::blob_length(llr_blob) / sizeof(uint8_t);

        if (dlen != SD_NUM)
            throw std::runtime_error("ERROR: invalid LLR data size");

        auto llrdata = (const uint8_t *) pmt::blob_data(llr_blob);
        for (int n = 0; n < SD_NUM; n++)
            d_llr_data[ch * SD_NUM + n] = (float) llrdata[n] / 8.0f;
    }

    // Check sequence number tag
    for (int ch = 0; ch < d_nstrm; ch++)
    {
        get_tags_in_window(d_tags, ch, 0, min_input_items,
                           pmt::string_to_symbol("seq"));
        if (d_tags.empty())
            throw std::runtime_error("ERROR: cannot find sequence number tag");

        uint64_t txseq = pmt::to_uint64(pmt::tuple_ref(d_tags[0].value, 0));
        uint64_t rxseq = pmt::to_uint64(pmt::tuple_ref(d_tags[0].value, 1));
        dout << "[Ch #" << ch << "] Tx Seq: " << txseq << " Rx Seq: " << rxseq << std::endl;
    }

    bool output_bits = output_items.size() > 1;
    bool output_majority = output_items.size() > 2;

    auto out0 = (float *) output_items[0];
    auto out1 = (uint8_t *) output_items[1];
    auto out2 = (uint8_t *) output_items[2];

    // LLR combining
    for (int idx = 0; idx < min_input_items; idx++)
    {
        float llr_sum = 0.0;
        float llr_sum_pos = 0.0, llr_sum_neg = 0.0;
        int maj_sum = 0;
        // sum LLR value for all streams
        for (int ch = 0; ch < d_nstrm; ch++)
        {
            auto in = (const uint8_t *) input_items[ch];
            int cur_modtype = (ch % 2) == 0 ? d_modtype1 : d_modtype2; // support two modulations only
            int llr_idx = idx / cur_modtype;
            int llr_offset = ch * SD_NUM + (llr_idx % SD_NUM);
            auto llrval = d_llr_data[llr_offset];
            if (!d_accurate_llr)
                // LLR approximation for high SNRs
                llr_sum += (in[idx] == 0) ? llrval : -llrval;
            else {
                // use more accurate LLR combining function
                if (in[idx] == 0 && llrval > 0) {
                    int llrdiff = abs(llrval - llr_sum_pos);
                    if (llrdiff <= 2.0)
                        llr_sum_pos += 1.0;
                    else
                        llr_sum_pos = std::max(llr_sum_pos, (float) llrval);
                } else if (llrval > 0) {
                    int llrdiff = abs(llrval - llr_sum_neg);
                    if (llrdiff <= 2.0)
                        llr_sum_neg += 1.0;
                    else
                        llr_sum_neg = std::max(llr_sum_neg, (float) llrval);
                }
            }
            if (d_majority)
                maj_sum += (in[idx] == 0) ? 1: -1;
        }
        if (d_accurate_llr)
            llr_sum = llr_sum_pos - llr_sum_neg;

        // soft LLR output
        out0[idx] = llr_sum;

        // hard-decision bits
        if (output_bits)
            out1[idx] = (llr_sum > 0.0) ? 0 : 1;

        // majority vote combining
        if (d_majority && output_majority)
            out2[idx] = (maj_sum > 0.0) ? 0 : 1;
    }

    return min_input_items;
}

} /* namespace gr::ncjt */
