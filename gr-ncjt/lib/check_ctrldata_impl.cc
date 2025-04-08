/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "check_ctrldata_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

check_ctrldata::sptr
check_ctrldata::make(int nstrm, int ctrl_datalen, bool has_txseq, bool has_rxseq,
                     bool has_llr, bool extract_llr, bool waitrdy, bool debug)
{
    return gnuradio::make_block_sptr<check_ctrldata_impl>(
        nstrm, ctrl_datalen, has_txseq, has_rxseq, has_llr, extract_llr, waitrdy, debug);
}

check_ctrldata_impl::check_ctrldata_impl(int nstrm, int ctrl_datalen, bool has_txseq, bool has_rxseq, bool has_llr,
                                         bool extract_llr, bool waitrdy, bool debug)
    : gr::tagged_stream_block(
    "check_ctrldata",
    gr::io_signature::make(nstrm, nstrm, sizeof(uint8_t)),
    gr::io_signature::make(nstrm, nstrm, sizeof(uint8_t)),
    "packet_len"),
      d_has_txseq(has_txseq), d_has_rxseq(has_rxseq), d_has_llr(has_llr), d_extract_llr(extract_llr),
      d_waitrdy(waitrdy), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 16)
        throw std::runtime_error("Invalid number of streams");
    d_nstrm = nstrm;

    d_infobuf_len = has_llr ? (32 + SD_NUM * LLR_WIDTH) : 32; // 16-bit seq. numbers, 6-bit LLRs for 52 subcarriers
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
    d_llr_vals = (uint8_t *) malloc_int(sizeof(int16_t) * SD_NUM * d_nstrm);
    if (d_info_buf == nullptr || d_cw_buf == nullptr || d_llr_data == nullptr || d_llr_vals == nullptr)
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
    }
    d_llr_data_valid = false;
    d_rxrdy = false;

    message_port_register_in(pmt::mp("rxrdy"));
    set_msg_handler(pmt::mp("rxrdy"), [this](const pmt::pmt_t &msg) { process_rxrdy_message(msg); });

    message_port_register_in(pmt::mp("llr"));
    set_msg_handler(pmt::mp("llr"), [this](const pmt::pmt_t &msg) { process_llr_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

check_ctrldata_impl::~check_ctrldata_impl()
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
check_ctrldata_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        noutput_items = std::min(noutput_items, ninput_items[ch]);
    return noutput_items;
}

void
check_ctrldata_impl::process_rxrdy_message(const pmt::pmt_t &msg) {
    bool rxrdy = pmt::to_bool(msg);
    if (d_rxrdy != rxrdy)
    {
        d_rxrdy = rxrdy;
        dout << "Update RX status: " << rxrdy << std::endl;
    }
}

void
check_ctrldata_impl::process_llr_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t csi_meta = pmt::car(msg);
    pmt::pmt_t csi_blob = pmt::cdr(msg);

    if (!pmt::dict_has_key(csi_meta, pmt::mp("llr")))
        throw std::runtime_error("invalid LLR data format");

    int dlen = pmt::blob_length(csi_blob) / sizeof(uint8_t);
    if (dlen == 0 || (dlen % SD_NUM) != 0)
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
            d_llr_vals[ch * SD_NUM + n] = llrval;
            for (int k=0; k < LLR_WIDTH; k++)
            {
                d_llr_data[offset + k] = (llrval & 0x20) >> 5; // MSB first
                llrval = (llrval << 1);
            }
        }
    dout << "Receive LLR data from message" << std::endl;
    d_llr_data_valid = true;
}

void
check_ctrldata_impl::encode_ctrl_data(int chan, uint8_t *outbuf)
{
    // clear buffer
    memset((void *) d_info_buf, 0, d_infobuf_len * sizeof(uint8_t));
    memset((void *) d_cw_buf, 0, d_ctrl_datalen * sizeof(uint8_t));

    // copy sequence numbers
    if (d_has_txseq)
    {
        uint16_t txseq = d_cur_txseq[chan];
        for (int k = 0; k < 16; k++)
        {
            d_info_buf[k] = (txseq & 0x8000) >> 15; // MSB first
            txseq = (txseq << 1);
        }
    }
    if (d_has_rxseq)
    {
        uint16_t rxseq = d_cur_rxseq[chan];
        for (int k = 0; k < 16; k++)
        {
            d_info_buf[k + 16] = (rxseq & 0x8000) >> 15; // MSB first
            rxseq = (rxseq << 1);
        }
    }

    // copy LLR data
    if (d_has_llr && d_extract_llr)
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
check_ctrldata_impl::decode_ctrl_data(int chan, const uint8_t *data, int datalen)
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
    if (decode_valid && d_has_txseq)
    {
        uint16_t txseq = 0;
        for (int k = 0; k < 16; k++)
            txseq = (txseq << 1) + d_info_buf[k]; // MSB first
        if (txseq)
        {
            d_cur_txseq[chan] = txseq;
            dout << "Decode TX SeqNo: " << txseq << std::endl;
        }
    }
    if (decode_valid && d_has_rxseq)
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
    if (decode_valid && d_has_llr && d_extract_llr && BCH_K * num_codewords >= (HDR_LEN + SD_NUM * LLR_WIDTH))
    {
        for (int n = 0; n < SD_NUM; n++)
        {
            uint8_t llrval = 0;
            for (int m = 0; m < LLR_WIDTH; m++)
            {
                int offset = LLR_WIDTH * n + m;
                d_llr_data[chan * LLR_WIDTH * SD_NUM + offset] = d_info_buf[HDR_LEN + offset];
                llrval += d_info_buf[HDR_LEN + offset] << (LLR_WIDTH - 1 - m);
            }
            d_llr_vals[chan * SD_NUM + n] = llrval;
        }
        d_llr_data_valid = true;
        dout << "Copy LLR data from input" << std::endl;
    }
}

int
check_ctrldata_impl::work(int noutput_items, gr_vector_int &ninput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
{
    int frame_data_len = ninput_items[0];
    for (int ch = 1; ch < d_nstrm; ch++)
        frame_data_len = std::min(frame_data_len, ninput_items[ch]);

    frame_data_len -= d_ctrl_datalen;
    if (frame_data_len <= 0)
        throw std::runtime_error("input data length too small");

    if (d_waitrdy && !d_rxrdy)
        return 0;

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        auto in = (const uint8_t *) input_items[ch];
        auto out = (uint8_t *) output_items[ch];

        if (d_has_rxseq || d_has_txseq || d_has_llr)
            decode_ctrl_data(ch, &in[0], d_ctrl_datalen);

        // add tags for sequence numbers
        auto seq_tags = pmt::make_tuple(pmt::from_uint64(d_cur_txseq[ch]), pmt::from_uint64(d_cur_rxseq[ch]));
        add_item_tag(ch, nitems_written(ch),
                     pmt::string_to_symbol("seq"),
                     seq_tags,
                     pmt::string_to_symbol(name()));

        // send LLR tag to downstream blocks
        if (d_extract_llr)
        {
            if (!d_llr_data_valid)
                memset((void *) &d_llr_vals[SD_NUM * ch], 0, sizeof(uint8_t) * SD_NUM);
            pmt::pmt_t llr_data = pmt::make_blob(&d_llr_vals[SD_NUM * ch], SD_NUM * sizeof(uint8_t));
            add_item_tag(ch, nitems_written(ch),
                         pmt::string_to_symbol("llr"),
                         llr_data,
                         pmt::string_to_symbol(name()));
        }

        // copy remaining data
        memcpy((void *) &out[0], &in[d_ctrl_datalen], sizeof(uint8_t) * frame_data_len);

        // add packet tags
        add_item_tag(ch, nitems_written(ch),
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(frame_data_len),
                     pmt::string_to_symbol(name()));
    }

    return frame_data_len;
}

} /* namespace gr::ncjt */
