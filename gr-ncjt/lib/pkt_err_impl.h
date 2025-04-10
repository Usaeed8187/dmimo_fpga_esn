/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_PKT_ERR_IMPL_H
#define INCLUDED_NCJT_PKT_ERR_IMPL_H

#include <gnuradio/ncjt/pkt_err.h>

namespace gr::ncjt
{

class pkt_err_impl : public pkt_err
{
private:
    int d_num_strms;              // number of channels
    bool d_shortterm;             // Output short-term stats
    int d_averaging_window;       // averaging window size for short-term stats
    char *d_pkt_data;             // transmitted packet data

    uint64_t d_pkt_len;           // packet length (number of bits)
    uint64_t d_total_pkts;        // total number of packets
    uint64_t d_cur_pos;           // current position in packet
    uint64_t d_cur_bit_errs[8];   // bit errors for current packet
    uint64_t d_cur_bit_errs_sum;  // total number of bit errors for current period
    uint64_t d_total_bit_errs[8]; // total number of bit errors
    uint64_t d_total_pkt_errs[8]; // total number of packet errors

    std::deque<double> d_ber_history;       // deque to store window of BERs
    double d_ber_history_sum;               // sum of BERs in the d_ber_history deque

    uint64_t
    read_packet_data(const char *filename);

    int d_log_freq;
    bool d_logpkterr;
    const bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    process_reset_message(const pmt::pmt_t &msg);

public:
    pkt_err_impl(int nstrms, const char *filename, bool shortterm, int avgwindow,
                 int logfreq, bool logpkterr, bool debug);
    ~pkt_err_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_PKT_ERR_IMPL_H */
