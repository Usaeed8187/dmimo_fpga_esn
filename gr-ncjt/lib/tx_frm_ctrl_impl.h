/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_TX_FRM_CTRL_IMPL_H
#define INCLUDED_NCJT_TX_FRM_CTRL_IMPL_H

#include <gnuradio/ncjt/tx_frm_ctrl.h>
#include <boost/thread/mutex.hpp>

namespace gr::ncjt
{

class tx_frm_ctrl_impl : public tx_frm_ctrl
{
private:
    const int SYM_LEN = 80; // OFDM symbol length (FFT+CP)

    int d_ntx;            // number of transmitter antennas
    int d_frame_length;   // full transmission frame length (beacon, data, and padding)
    int d_beacon_len;     // beacon length in samples
    int d_data_length;    // data symbols length in samples
    int d_padding_length; // zero padding length in samples

    bool d_txen;  // indicate whether transmission is enabled
    bool d_first_burst; // indicate first transmission bust
    double d_repeat_interval;  // repeat transmission interval
    uint64_t d_frame_interval; // frame interval in samples
    int d_pkts_per_sec; // packets per second
    double d_samplerate; // sampling rate
    int d_delay; // extra sample delay (for debugging)

    uint64_t d_txtime_start;  // start time in seconds
    double d_txtime_offset;  // transmission time offset relative to frame start position
    double d_txtime_adjustment; // current txtime adjustment
    double d_clk_offset_est; // clock offset estimate
    uint64_t d_cur_frame_cnt;  // total frame counter
    bool d_frame_cnt_adjusted; // frame counter adjusted
    uint64_t d_prev_frame_start; // previous frame start offset
    uint64_t d_prev_frame_cnt; // previous frame count
    uint64_t d_time_secs;  // integer seconds of next transmission time
    double d_time_fracs;  // fractional seconds of next transmission time

    gr_complex *d_beacon_data;  // preamble data

    boost::mutex fp_mutex;
    const bool d_debug;
    pmt::pmt_t _id;

    int
    read_beacon_data(const char *filename);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    process_rxtime_message(const pmt::pmt_t &msg);

    void
    process_txen_message(const pmt::pmt_t &msg);

public:
    tx_frm_ctrl_impl(int ntx, int ndatasyms, const char *filename, double samplerate, int pktspersec,
                     double starttime, int padding, bool autostart, int delay, bool debug);
    ~tx_frm_ctrl_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRM_CTRL_IMPL_H */
