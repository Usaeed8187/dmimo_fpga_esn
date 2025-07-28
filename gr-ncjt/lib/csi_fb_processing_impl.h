/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_CSI_FB_PROCESSING_IMPL_H
#define INCLUDED_NCJT_CSI_FB_PROCESSING_IMPL_H

#include <gnuradio/ncjt/csi_fb_processing.h>
#include <gnuradio/message.h>
#include "cmatrix.h"
#include <gnuradio/fec/polar_decoder_sc.h>
#include <vector>
#include <array>

namespace gr {
namespace ncjt {

class csi_fb_processing_impl : public csi_fb_processing {
private:
    int d_nrb;     // Number of RBs for feedback
    int d_nss;  // Number of uplink spatial streams
    int d_ntx_gnb;  // Number of gNB transmitter antennas
    bool d_wideband; // Use wideband precoding method
    int d_logfreq;
    int d_total_frames;

    int d_pmi_bits;             // number of bits corresponding to pmi index in csi feedback
    int d_cqi_bits;             // number of bits corresponding to cqi index in csi feedback
    int d_crc_bits;             // number of bits allocated to crc check in csi feedback
    int d_total_csi_bits;       // total (coded) bits in csi feedback
    int d_num_ue;               // number of UEs in the TX Squad
    int d_rx_modtype;           // modulation order of CSI FB
    float d_polar_code_rate;    // polar coding rate used in csi feedback
    bool d_polar_inited {false};
    std::shared_ptr<gr::fec::code::polar_decoder_sc> d_polar_dec;
    static const int REL_SEQ_128[128];
    std::vector<std::array<gr_complex,8>> d_codebook;
    std::vector<uint8_t> d_wb_cqi; // decoded wideband CQI per stream
    std::vector<float>   d_wb_cqi_db; // CQI threshold in dB per stream

    bool d_debug;

protected:

    boost::mutex fp_mutex;

    void
    build_codebook();

    void
    init_polar();

    uint16_t
    compute_crc10(const std::vector<uint8_t> &bits) const;

    int calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
  csi_fb_processing_impl(int num_ue, int rx_modtype, int nss, int ntx_gnb, bool wideband, int nrb,
                         int pmi_bits, int cqi_bits, int crc_bits,
                         int total_csi_bits, float polar_code_rate, bool debug);
  ~csi_fb_processing_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace ncjt
} // namespace gr

#endif /* INCLUDED_NCJT_CSI_FB_PROCESSING_IMPL_H */
