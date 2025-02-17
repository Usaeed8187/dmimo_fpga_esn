/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_LTF_CHANEST_IMPL_H
#define INCLUDED_NCJT_LTF_CHANEST_IMPL_H

#include <gnuradio/ncjt/ltf_chanest.h>
#include "cmatrix.h"

namespace gr::ncjt
{

class ltf_chanest_impl : public ltf_chanest
{
private:
    const int MAX_NSS = 8; // maximum number of streams/antennas supported
    int d_fftsize; // OFDM FFT size (64 or 256)
    int d_scnum;  // number of valid subcarriers
    int d_ntx;  // number of transmitter antennas
    int d_nss;  // number of transmitter data streams
    int d_nrx;  // number of receive antennas
    int d_npt; // number of tracking pilots per symbol
    int d_preamble_symbols; // total number of HT preamble symbols
    int d_data_symbols;        // total number of data symbols per packet
    int d_last_sym, d_cur_sym; // previous and current OFDM symbol index
    float d_cpe_phi;           // CPE in radian
    float d_sigpwr_est;        // signal power estimation
    float d_noise_est;         // noise power estimation
    double d_sigpwr_sum;       // signal power sum
    double d_noise_sum;        // noise power sum

    unsigned d_pilot_lsfr; // LSFR state for pilot parity sequence
    float d_cur_pilot[8][8]; // pilots for current OFDM symbol (mode 4 or 8)
    gr_complex *d_chan_est; // channel estimate for data reception (Nt,Nr,Nsc)
    gr_complex *d_chan_csi; // channel estimation for CSI feedback (Nt,Nr,Nsc)
    gr_complex *d_cshift; // cyclic shift compensation
    CMatrixX d_Pd; // P matrix for MMSE detection
    static const gr_vector_float NORM_LTF_SEQ_64; // normalized LTF sequence
    static const gr_vector_float NORM_LTF_SEQ_256; // normalized LTF sequence
    gr_vector_float NORM_LTF_SEQ; // current normalized LTF sequence

    bool d_csi_en; // enable CSI feedback
    bool d_remove_cyclic_shift; // remove cyclic shift from channel estimation
    int d_total_frames; // total number of data frames
    int d_reset_frames; // reset frame counter

    pmt::pmt_t _id;
    const int d_logfreq;
    const bool d_debug;

    void
    ltf_chan_est_1tx(gr_vector_const_void_star &input_items, gr_vector_void_star &output_items, int output_offset);

    void
    ltf_chan_est_2rx(gr_vector_const_void_star &input_items, gr_vector_void_star &output_items, int output_offset);

    void
    ltf_chan_est_nrx(gr_vector_const_void_star &input_items, gr_vector_void_star &output_items, int output_offset);

    void
    csi_chan_est_nrx(gr_vector_const_void_star &input_items, int input_offset);

    void
    update_pilots(int symidx);

    void
    cpe_estimate_comp(gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items,
                      int nsymcnt,
                      int noutsymcnt);

    void
    process_reset_message(const pmt::pmt_t &msg);

    void
    send_cpe_message();

    void
    send_snr_message();

    void
    send_csi_message();

    void
    add_frame_tag(uint64_t offset);

    void
    add_packet_tag(uint64_t offset, int packet_len);

    void
    add_power_noise_tag(uint64_t offset, float signal_pwr, float noise_est);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    ltf_chanest_impl(int fftsize, int ntx, int nrx, int npreamblesyms, int ndatasyms,
                     bool csifb, bool removecs, int logfreq, bool debug);
    ~ltf_chanest_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_LTF_CHANEST_IMPL_H */
