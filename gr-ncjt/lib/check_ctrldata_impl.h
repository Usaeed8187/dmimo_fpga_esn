/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_CHECK_CTRLDATA_IMPL_H
#define INCLUDED_NCJT_CHECK_CTRLDATA_IMPL_H

#include <gnuradio/ncjt/check_ctrldata.h>
#include <gnuradio/ncjt/bchclass.h>

namespace gr::ncjt
{

class check_ctrldata_impl : public check_ctrldata
{
private:
    const int SD_NUM = 52; // number of data subcarriers
    const int HDR_LEN = 32; // header length (2 sequence numbers)
    const int LLR_WIDTH = 6; // data width for LLRs
    const int BCH_N = 31; // BCH codeword length
    const int BCH_K = 11; // BCH info length
    int d_nstrm; // number of streams
    bool d_has_txseq; // add sequence numbers at TxUE
    bool d_has_rxseq; // add sequence numbers at RxUE
    bool d_has_llr; // add LLR data at RxUE
    bool d_extract_llr; // output extracted LLR
    bool d_waitrdy; // wait for ready status
    bool d_llr_data_valid; // has received valid LLR
    bool d_rxrdy; // receiver ready status

    int d_ctrl_datalen;  // total control channel length (in bits)
    int d_infobuf_len;   // info buffer length
    int d_num_codewords; // number of BCH codewords
    int d_cwbuf_len;     // codeword buffer length

    uint16_t d_cur_txseq[16];  // 16-bit sequence number from TxUE
    uint16_t d_cur_rxseq[16];  // 16-bit sequence number from RxUE
    uint8_t *d_info_buf;  // information buffer
    uint8_t *d_cw_buf;  // information buffer
    uint8_t *d_llr_data; // binary data for 6-bit LLR values
    uint8_t *d_llr_vals; // 8-bit LLR values for output
    uint8_t *d_scramble_seq; // scrambling sequence

    bchclass *bch;

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    encode_ctrl_data(int chan, uint8_t *outbuf);

    void
    decode_ctrl_data(int chan, const uint8_t *data, int datalen);

    void
    process_rxrdy_message(const pmt::pmt_t &msg);

    void
    process_llr_message(const pmt::pmt_t &msg);

public:
    check_ctrldata_impl(int nstrm, int ctrl_datalen, bool has_txseq, bool has_rxseq,
                        bool has_llr, bool extract_llr, bool waitrdy, bool debug);
    ~check_ctrldata_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_CHECK_CTRLDATA_IMPL_H */
