/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RX_SYNC_IMPL_H
#define INCLUDED_NCJT_RX_SYNC_IMPL_H

#include <gnuradio/ncjt/rx_sync.h>
#include <gnuradio/filter/fir_filter.h>

namespace gr::ncjt
{

class rx_sync_impl : public rx_sync
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
    const int MAX_CHANS = 20;  // Maximum number of IQ channels
    const int PEAK_THRD = 5;   // Minimum peak duration of auto-correlation windows

    int d_num_chans;  // Total number of IQ data channels
    int d_frame_len1;  // frame length in samples (HT preamble + data symbols)
    int d_ht_len1;  // HT beacon length
    double d_samplerate;  // Baseband sampling frequency
    int d_frame_interval;   // packet repeat interval (in samples)
    int d_wait_interval;  // Wait interval between packets (in number of IQ samples)
    int64_t d_rxtime_offset; // Rx time difference from tag[offset] samples
    bool d_use_lltf2; // Use alternative L-LTF for synchronization

    bool d_p2rxue;  // Phase 2 RxUE mode
    int d_frame_len2;  // Phase 2 frame length in samples (HT preamble + data symbols)
    int d_ht_len2; // Phase 2 HT beacon length
    uint64_t d_p2start_offset; // Phase 2 frame start offset
    bool d_skip_p2_frame; // skip phase-2 deframing

    float d_rxpwr_thrd;  // Receiver power threshold for signal detection
    float d_acorr_thrd;  // Auto-correlation detection threshold
    float d_xcorr_thrd;  // Cross-correlation detection threshold
    int d_max_corr_len;  // Maximal auto-correlation buffer length

    int d_rx_ready_cnt1, d_rx_ready_cnt2;  // receiver synchronization counter
    int d_sync_err_cnt1, d_sync_err_cnt2;  // fine synchronization errors

    int d_corr_buf_pos;
    float d_current_foe_comp1, d_current_foe_comp2;
    float d_fine_foe_comp1, d_fine_foe_comp2;
    int d_fine_foe_cnt1, d_fine_foe_cnt2;
    int d_data_samples;
    int d_wait_count;

    const int CLK_EST_SAMPLES = 200; // number of samples for clock offset estimation
    uint64_t d_prev_p2frame_start, d_next_p2frame_start;
    uint64_t d_prev_frame_start, d_next_frame_start;
    int64_t d_clk_offset_sum;
    int d_clk_offset_cnt;
    double d_clk_offset_est;
    bool d_clk_offset_ok;

    float *d_pwrest_buf;
    gr_complex *d_corr_buf;

    static const std::vector<gr_complex> LTF_SEQ_1;
    static const std::vector<gr_complex> LTF_SEQ_2;
    gr::filter::kernel::fir_filter_ccc d_xcorr_fir_1, d_xcorr_fir_2;
    gr_complex *d_xcorr_buf;
    gr_complex *d_input_buf;
    float *d_xcorr_val;

    const bool d_debug;
    pmt::pmt_t _id;

    enum { P1SEARCH, P1FINESYNC, P1DEFRAME, WAIT1, P2SEARCH, P2FINESYNC, P2DEFRAME, WAIT2} d_state;

    int
    sync_search(const gr_vector_const_void_star &input_items, int buffer_len, float &current_foe_comp);

    int
    fine_sync(const gr_vector_const_void_star &input_items, int buffer_len, bool lltfv2,
              float &current_foe_comp, float &fine_foe_comp, int &rx_ready_cnt, int &fine_foe_cnt);

    void
    send_tagcmd();

    void
    send_rxstate(bool ready);

    void
    check_rxtime(int rx_windows_size);

    void
    send_rxtime();

public:
    rx_sync_impl(int nchans, int npreamblesyms, int ndatasyms, double sampling_freq, int pktspersec,
                 bool p2rxue, int p2preamblelen, int p2framelen, double p2_start, double rxpwr_thrd,
                 double acorr_thrd, double xcorr_thrd, int max_corr_len, bool lltf2, bool debug);
    ~rx_sync_impl();

    void
    forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int
    general_work(int noutput_items, gr_vector_int &ninput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RX_SYNC_IMPL_H */
