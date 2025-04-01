/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "add_ctrldata_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

add_ctrldata::sptr
add_ctrldata::make(
    int nstrm, int ctrl_datalen, bool add_txseq, bool add_rxseq, bool add_llr, bool debug)
{
    return gnuradio::make_block_sptr<add_ctrldata_impl>(
        nstrm, ctrl_datalen, add_txseq, add_rxseq, add_llr, debug);
}

add_ctrldata_impl::add_ctrldata_impl(
    int nstrm, int ctrl_datalen, bool add_txseq, bool add_rxseq, bool add_llr, bool debug)
    : gr::tagged_stream_block(
    "add_ctrldata",
    gr::io_signature::make(nstrm, nstrm, sizeof(uint8_t)),
    gr::io_signature::make(nstrm, nstrm, sizeof(uint8_t)),
    "packet_len"), d_add_txseq(add_txseq), d_add_rxseq(add_rxseq), d_add_llr(add_llr), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 16)
        throw std::runtime_error("Invalid number of streams");
    d_nstrm = nstrm;

    d_infobuf_len = add_llr ? (32 + SD_NUM * LLR_WIDTH) : 32; // 16-bit seq. numbers, 6-bit LLRs for 52 subcarriers
    d_num_codewords = (int) ceil(d_infobuf_len / (double) BCH_K);  // BCH(31,11)
    d_infobuf_len = BCH_K * d_num_codewords;
    d_cwbuf_len = BCH_N * d_num_codewords;

    if (ctrl_datalen < d_cwbuf_len)
        throw std::runtime_error("control data length too small");
    if (ctrl_datalen % SD_NUM != 0)
        throw std::runtime_error("control data length not correct");
    d_ctrl_datalen = ctrl_datalen;

    bch = new bchclass(BCH31_11);

    d_info_buf = (uint8_t *) malloc_int(sizeof(int16_t) * BCH_K * 32);
    d_cw_buf = (uint8_t *) malloc_int(sizeof(int16_t) * BCH_N * 32);
    d_llr_data = (uint8_t *) malloc_int(sizeof(int16_t) * LLR_WIDTH * SD_NUM * d_nstrm);
    if (d_info_buf == nullptr || d_cw_buf == nullptr || d_llr_data == nullptr)
        throw std::runtime_error("memory allocation error");

    // generate scrambling sequence
    d_scramble_seq = (uint8_t *) malloc_int(sizeof(int16_t) * d_ctrl_datalen);
    unsigned scr_lsfr = 0x78;
    for (int k = 0; k < d_ctrl_datalen; k++)
    {
        unsigned parity_bit = (scr_lsfr >> 6) ^ ((scr_lsfr >> 3) & 0x1);
        scr_lsfr = ((scr_lsfr << 1) & 0x7E) | parity_bit; // circular 7-bit shifter
        d_scramble_seq[k] = parity_bit;
    }

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        d_cur_txseq[ch] = 0;
        d_cur_rxseq[ch] = 0;
        for (int n = 0; n < SD_NUM; n++)
            {
                int offset = ch * LLR_WIDTH * SD_NUM + LLR_WIDTH * n;
                uint8_t llrval = 27; // default test val
                for (int k=0; k < LLR_WIDTH; k++)
                {
                    d_llr_data[offset + k] = (llrval & 0x20) >> 5; // MSB first
                    llrval = (llrval << 1);
                }
            }
    }

    message_port_register_in(pmt::mp("llr"));
    set_msg_handler(pmt::mp("llr"), [this](const pmt::pmt_t &msg) { process_llr_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

add_ctrldata_impl::~add_ctrldata_impl()
{
    if (d_info_buf != nullptr)
        free(d_info_buf);
    if (d_cw_buf != nullptr)
        free(d_cw_buf);
    if (d_llr_data != nullptr)
        free(d_llr_data);
    if (d_scramble_seq != nullptr)
        free(d_scramble_seq);
}

int
add_ctrldata_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = d_ctrl_datalen + ninput_items[0];
    return noutput_items;
}

void
add_ctrldata_impl::process_llr_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t csi_meta = pmt::car(msg);
    pmt::pmt_t csi_blob = pmt::cdr(msg);

    if (!pmt::dict_has_key(csi_meta, pmt::mp("llr")))
        throw std::runtime_error("invalid LLR data format");

    int dlen = pmt::blob_length(csi_blob) / sizeof(uint8_t);
    if (dlen != SD_NUM)
    {
        dout << "LLR data length: " << dlen << std::endl;
        throw std::runtime_error("invalid LLR data length");
    }
    // assuming same CSI for all streams at the moment
    auto llrdata = (const uint8_t *) pmt::blob_data(csi_blob);
    for (int ch = 0; ch < d_nstrm; ch++)
        for (int n = 0; n < SD_NUM; n++)
        {
            int offset = ch * LLR_WIDTH * SD_NUM + LLR_WIDTH * n;
            uint8_t llrval = llrdata[n];
            for (int k=0; k < LLR_WIDTH; k++)
            {
                d_llr_data[offset + k] = (llrval & 0x20) >> 5; // MSB first
                llrval = (llrval << 1);
            }
        }
}

void
add_ctrldata_impl::encode_ctrl_data(int chan, uint8_t *outbuf)
{
    // clear buffer
    memset((void *) d_info_buf, 0, d_infobuf_len * sizeof(uint8_t));
    memset((void *) d_cw_buf, 0, d_ctrl_datalen * sizeof(uint8_t));

    // copy sequence numbers
    uint16_t txseq = d_cur_txseq[chan];
    for (int k = 0; k < 16; k++)
    {
        d_info_buf[k] = (txseq & 0x8000) >> 15; // MSB first
        txseq = (txseq << 1);
    }
    uint16_t rxseq = d_cur_rxseq[chan];
    for (int k = 0; k < 16; k++)
    {
        d_info_buf[k + 16] = (rxseq & 0x8000) >> 15; // MSB first
        rxseq = (rxseq << 1);
    }

    // copy LLR data
    if (d_add_llr)
    {
        for (int n = 0; n < SD_NUM * LLR_WIDTH; n++)
            d_info_buf[HDR_LEN + n] = d_llr_data[chan * LLR_WIDTH * SD_NUM + n];
    }

    // BCH encoding
    for (int k = 0; k < d_num_codewords; k++)
    {
        bch->encode_bch(&d_info_buf[k * BCH_K], &d_cw_buf[k * BCH_N]);
    }

    // interleaving and scrambling
    int intlv_width = d_ctrl_datalen / SD_NUM;
    for (int m = 0; m < intlv_width; m++)
        for (int n = 0; n < SD_NUM; n++)
        {
            int buffer_addr = n * intlv_width + m;
            int output_addr = m * SD_NUM + n;
            outbuf[output_addr] = d_cw_buf[buffer_addr] ^ d_scramble_seq[output_addr];
        }
}

void
add_ctrldata_impl::decode_ctrl_data(int chan, const uint8_t *data, int datalen)
{
    // de-interleaving and de-scrambling
    int intlv_width = datalen / SD_NUM;
    for (int m = 0; m < intlv_width; m++)
        for (int n = 0; n < SD_NUM; n++)
        {
            int input_addr = m * SD_NUM + n;
            int buffer_addr = n * intlv_width + m;
            d_cw_buf[buffer_addr] = data[input_addr] ^ d_scramble_seq[input_addr];
        }

    // BCH decoding
    int num_codewords = floor(datalen / 31.0);
    bool decode_valid = true;
    for (int k = 0; k < num_codewords; k++)
    {
        int ret = bch->decode_bch(&d_cw_buf[k * BCH_N], &d_info_buf[k * BCH_K]);
        if (ret < 0)
            decode_valid = false;
    }

    // save sequence numbers
    if (!d_add_txseq)
    {
        uint16_t txseq = 0;
        for (int k = 0; k < 16; k++)
            txseq = (txseq << 1) + d_info_buf[k]; // MSB first
        if (txseq > 0)
        {
            d_cur_txseq[chan] = txseq;
            dout << "Decode TX SeqNo: " << txseq << std::endl;
        }
    }
    if (!d_add_rxseq)
    {
        uint16_t rxseq = 0;
        for (int k = 0; k < 16; k++)
            rxseq = (rxseq << 1) + d_info_buf[k + 16]; // MSB first
        if (rxseq > 0)
        {
            d_cur_rxseq[chan] = rxseq;
            dout << "Decode RX SeqNo: " << rxseq << std::endl;
        }
    }

    // save LLR data
    if (decode_valid && BCH_K * num_codewords >= (HDR_LEN + SD_NUM * LLR_WIDTH))
    {
        for (int n = 0; n < SD_NUM * LLR_WIDTH; n++)
            d_llr_data[chan * LLR_WIDTH * SD_NUM + n] = d_info_buf[HDR_LEN + n];
        dout << "Copy LLR data from input" << std::endl;
    }
}

int
add_ctrldata_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    int frame_data_len = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        frame_data_len = std::min(frame_data_len, ninput_items[ch]);

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        // check if control data is present
        int ctrl_input_len = 0;
        std::vector<gr::tag_t> d_tags;
        get_tags_in_window(d_tags, 0, 0, 1,
                           pmt::string_to_symbol("ctrl_data"));
        if (!d_tags.empty())
            ctrl_input_len = pmt::to_long(d_tags[0].value);

        auto in = (const uint8_t *) input_items[ch];
        auto out = (uint8_t *) output_items[ch];

        // decode sequence numbers and copy LLRs values if present
        if (ctrl_input_len > 0)
        {
            frame_data_len -= ctrl_input_len;
            decode_ctrl_data(ch, &in[0], ctrl_input_len);
            // dout << "AddControlData: received upstream control data" << std::endl;
        }

        // update sequence numbers
        if (d_add_txseq)
            d_cur_txseq[ch] += 1;
        if (d_add_rxseq)
            d_cur_rxseq[ch] += 1;

        // generate control signals
        encode_ctrl_data(ch, &out[0]);
        // copy remaining data
        memcpy((void *) &out[d_ctrl_datalen], &in[ctrl_input_len], sizeof(uint8_t) * frame_data_len);

        // add tags
        add_item_tag(ch, nitems_written(ch),
                     pmt::string_to_symbol("ctrl_data"),
                     pmt::from_long(d_ctrl_datalen),
                     pmt::string_to_symbol(name()));

        // add packet tags
        add_item_tag(ch, nitems_written(ch),
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(d_ctrl_datalen + frame_data_len),
                     pmt::string_to_symbol(name()));
    }

    return (d_ctrl_datalen + frame_data_len);
}

} /* namespace gr::ncjt */
