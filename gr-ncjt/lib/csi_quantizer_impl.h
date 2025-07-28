/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 */

#ifndef INCLUDED_NCJT_CSI_QUANTIZER_IMPL_H
#define INCLUDED_NCJT_CSI_QUANTIZER_IMPL_H

#include <gnuradio/ncjt/csi_quantizer.h>
#include "cmatrix.h"
#include <gnuradio/fec/polar_encoder.h>
#include <vector>
#include <array>
#include <mutex>

namespace gr::ncjt {

class csi_quantizer_impl : public csi_quantizer
{
private:
    static constexpr int MAX_NSS = 4; // maximum number of streams/antennas supported
    int d_fftsize; // OFDM FFT size (64 or 256)
    int d_scnum; // number of valid subcarriers
    int d_sdnum; // number of data subcarriers
    int d_modtype; // modulation order expected by downstream blocks
    int d_ntx; // number of transmitter antennas
    int d_nss; // number of transmitter spatial streams
    int d_nrx; // number of receiver antennas
    int d_npt; // number of tracking pilots per symbol
    int d_rbg_size; // number of subcarriers in one RB
    int d_ofdm_syms_fb; // number of ofdm symbols used in feedback (as expected by downstream blocks)
    int d_logfreq; // number of frames after which a dout will be performed
    int d_total_frames; // total number of data frames
    int d_exp_bits; // bits expected by down-stream blocks
    bool d_debug; // debugging on or off

    std::vector<gr_complex> d_chan_csi;
    float d_noise_est {1e-5f};

    uint8_t d_wb_pmi_idx {0};
    std::vector<std::array<gr_complex, 8>> d_codebook;

    bool d_polar_inited {false};
    std::shared_ptr<gr::fec::code::polar_encoder> d_polar_enc;
    static const int REL_SEQ_128[128];

    std::vector<uint8_t> d_rbg_cqi;
    std::vector<uint8_t> d_wb_cqi;

    std::vector<uint8_t> d_out_buff;
    bool d_add_tag {false};
    std::mutex d_mutex;
    pmt::pmt_t d_id;

    void build_codebook();
    void init_polar();
    uint16_t compute_crc10(const std::vector<uint8_t>& bits) const;
    void compute_quantised_csi();
    void process_csi_message(const pmt::pmt_t& msg);

    void
    add_packet_tag(uint64_t offset, int packet_len);

public:
    csi_quantizer_impl(int fftsize,
                       int ntx,
                       int nrx,
                       int nss,
                       int rbg_size,
                       int n_ofdm_syms_fb,
                       int logfreq,
                       bool debug);
    ~csi_quantizer_impl() override;

    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_CSI_QUANTIZER_IMPL_H */