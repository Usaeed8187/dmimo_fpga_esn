/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_GNB_SYNC_IMPL_H
#define INCLUDED_NCJT_GNB_SYNC_IMPL_H

#include <gnuradio/ncjt/gnb_sync.h>
#include <gnuradio/filter/fir_filter.h>

namespace gr::ncjt
{

class gnb_sync_impl : public gnb_sync
{
private:
    const int FFT_LEN = 64;      // FFT length
    const int SYM_LEN = 80;      // OFDM symbol length
    const int STF_LEN = 160;     // L-STF length, 16 x 10 = 160 samples
    const int LTF_LEN = 160;     // L-LTF length, 64 x 2 + 32 = 160 samples
    const int CORR_DELAY = 16;   // L-STF symbol length (16)
    const int CORR_WINDOW = 48;  // Auto-corr window size
    const int CORR_BUF_LEN = 64; // Ring buffer length
    const int XCORR_DATA_LEN = LTF_LEN * 4; // Cross-correlation data buffer length
    const int MAX_XCORR_LEN = 1024;   // Maximum cross-correlation output buffer length
    const int MAX_PREAMBLE_SYMS = 12; // Maximal number of HT preamble symbols (HT-SIG, etc.)
    const int MAX_CHANS = 8;   // Maximum number of IQ channels
    const int PEAK_THRD = 5;   // Minimum peak duration of auto-correlation windows

    int d_num_chans;         // Total number of IQ data channels
    double d_samplerate;     // Baseband sampling frequency
    double d_pkt_interval;   // packet repeat interval (in seconds)
    int d_hw_delay_p2;       // Estimated USRP TX/RX processing delay in samples (Phase 2)
    int d_hw_delay_p3;       // Estimated USRP TX/RX processing delay in samples (Phase 3)
    int64_t d_rxtime_offset; // Rx time difference from tag[offset] samples

    float d_rxpwr_thrd;  // Receiver power threshold for signal detection
    float d_acorr_thrd;  // Auto-correlation detection threshold
    float d_xcorr_thrd;  // Cross-correlation detection threshold
    int d_max_corr_len;  // Maximal auto-correlation buffer length

    uint64_t d_frame_interval;  // frame interval in samples
    uint64_t d_p2_offset;  // phase 2 transmission start position
    uint64_t d_p3_offset;  // phase 3 transmission start position
    int d_p2_htlen;        // P2 HT preamble length (excluding HT-LTFs)
    int d_p3_htlen;        // P3 HT preamble length (excluding HT-LTFs)
    int d_frame_len_1;     // P2 frame length in samples (HT preamble + data symbols)
    int d_frame_len_2;     // P3 frame length in samples (HT preamble + data symbols)
    int d_wait_interval0;  // wait interval for initial synchronization
    int d_wait_interval1;  // wait interval between P2 and P3 reception
    int d_wait_interval2;  // wait interval between P3 and new cycle

    uint64_t d_frame_start1; // frame start position for P2
    uint64_t d_frame_start2; // frame start position for P3
    int d_rx_ready_cnt1;  // receiver synchronization counter
    int d_rx_ready_cnt2;  // receiver synchronization counter
    int d_sync_err_cnt1;  // receiver synchronization counter
    int d_sync_err_cnt2;  // receiver synchronization counter
    bool d_skip_p2_frame; // skip current P2 frame reception
    bool d_skip_p3_frame;  // skip current P3 frame reception

    int d_corr_buf_pos;
    float d_current_foe_comp1, d_current_foe_comp2;
    float d_fine_foe_comp1, d_fine_foe_comp2;
    int d_fine_foe_cnt1, d_fine_foe_cnt2;
    int d_data_samples;
    int d_wait_count;

    float *d_pwrest_buf;
    gr_complex *d_corr_buf;

    static const std::vector<gr_complex> LTF_SEQ;
    gr::filter::kernel::fir_filter_ccc d_xcorr_fir;
    gr_complex *d_xcorr_buf;
    gr_complex *d_input_buf;
    float *d_xcorr_val;

    const bool d_debug;
    pmt::pmt_t _id;

    enum
    {
        RXTIME, P2SEARCH, P2SYNC, P3SEARCH, P3SYNC, P2DEFRAME, P3DEFRAME, WAIT0, WAIT1, WAIT2
    } d_state;

    int
    sync_search(const gr_vector_const_void_star &input_items, int buffer_len,
                float &current_foe_comp);

    int
    fine_sync(const gr_vector_const_void_star &input_items, int buffer_len,
              float &current_foe_comp, float &fine_foe_comp, int &rx_ready_cnt, int &fine_foe_cnt);

    void
    send_tagcmd();

    void
    send_rxstate(bool ready);

    void
    check_rxtime(int rx_windows_size);

public:
    gnb_sync_impl(int nchans, double samplerate, int pktspersec, int hwdelayp2, int hwdelayp3,
                  int p2_htlen, int p2_datalen, int p3_htlen, int p3_datalen, double p2_start, double p3_start,
                  double rxpwr_thrd, double acorr_thrd, double xcorr_thrd, int max_corr_len, bool debug);
    ~gnb_sync_impl();

    void
    forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int
    general_work(int noutput_items, gr_vector_int &ninput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_GNB_SYNC_IMPL_H */
