/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_DUAL_TX_CTRL_IMPL_H
#define INCLUDED_NCJT_DUAL_TX_CTRL_IMPL_H

#include <gnuradio/ncjt/dual_tx_ctrl.h>
#include <boost/thread/mutex.hpp>

namespace gr::ncjt
{

class dual_tx_ctrl_impl : public dual_tx_ctrl
{
private:
    int d_ntx;             // number of transmitter antennas
    int d_frame_length1;   // full transmission frame length (beacon, data, and padding)
    int d_beacon_len1;     // beacon length in samples
    int d_data_length1;    // data symbols length in samples
    int d_frame_length2;   // full transmission frame length (beacon, data, and padding)
    int d_beacon_len2;     // beacon length in samples
    int d_data_length2;    // data symbols length in samples
    int d_padding_length;  // zero padding length in samples

    bool d_txen;  // indicate whether transmission is enabled
    double d_repeat_interval;  // repeat transmission interval
    uint64_t d_frame_interval; // frame interval in samples
    int d_pkts_per_sec; // packets per second
    double d_samplerate; // sampling rate

    uint64_t d_txtime_start;  // start time in seconds
    double d_txtime_offset1;  // transmission time offset relative to frame start position
    double d_txtime_offset2;  // transmission time offset relative to frame start position
    double d_txtime_adjustment; // current txtime adjustment
    double d_clk_offset_est; // clock offset estimate
    uint64_t d_cur_frame_cnt;  // total frame counter
    bool d_frame_cnt_adjusted; // frame counter adjusted
    uint64_t d_prev_frame_start; // previous frame start offset
    uint64_t d_prev_frame_cnt; // previous frame count
    uint64_t d_time_secs;  // integer seconds of next transmission time
    double d_time_fracs;  // fractional seconds of next transmission time

    gr_complex *d_beacon_data1;  // preamble data
    gr_complex *d_beacon_data2;  // preamble data

    boost::mutex fp_mutex;
    const bool d_debug;
    pmt::pmt_t _id;

    gr_complex *
    read_beacon_data(const char *filename, int *beacon_len);

protected:
    void
    process_rxtime_message(const pmt::pmt_t &msg);

    void
    process_txen_message(const pmt::pmt_t &msg);

    void
    add_packet_tags(uint64_t offset, int pkt_len, const pmt::pmt_t &tx_time);

public:
    dual_tx_ctrl_impl(int ntx, double samplerate, int pktspersec, int framelen1,
                      const char *beaconfile1, double starttime1, int framelen2,
                      const char *beaconfile2, double starttime2, int padding,
                      bool autostart, bool debug);
    ~dual_tx_ctrl_impl();

    void
    forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int
    general_work(int noutput_items, gr_vector_int &ninput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_DUAL_TX_CTRL_IMPL_H */
