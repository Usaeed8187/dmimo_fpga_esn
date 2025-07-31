/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "csi_fb_processing_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <Eigen/Dense>
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Eigenvalues>

namespace gr {
namespace ncjt {

const int csi_fb_processing_impl::REL_SEQ_128[128] = {
    33, 34, 35, 36, 37, 38, 40, 41, 42, 44, 48, 49, 76, 52, 56, 0,
    65, 66, 67, 68, 69, 70, 72, 73, 74, 50, 1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12, 32, 14, 16, 17, 18, 19, 20, 21, 22, 24,
    25, 26, 28, 13, 64, 112, 96, 80, 81, 82, 104, 84, 88, 97, 100, 98,
    15, 23, 27, 29, 39, 30, 43, 45, 46, 51, 71, 53, 75, 54, 77, 57,
    83, 78, 58, 85, 86, 60, 89, 99, 90, 101, 102, 92, 105, 106, 108, 113,
    114, 116, 120, 31, 47, 55, 79, 59, 87, 61, 91, 62, 103, 93, 107, 94,
    109, 115, 110, 117, 118, 121, 122, 124, 63, 95, 111, 119, 123, 125, 126, 127
};

csi_fb_processing::sptr
csi_fb_processing::make(int num_ue, int rx_modtype, int nss, int ntx_gnb, bool wideband, int nrb,
                        int pmi_bits, int cqi_bits, int crc_bits,
                        int total_csi_bits, float polar_code_rate, bool debug) {
  return gnuradio::make_block_sptr<csi_fb_processing_impl>(
      num_ue, rx_modtype, nss, ntx_gnb, wideband, nrb, pmi_bits, cqi_bits, crc_bits, total_csi_bits,
      polar_code_rate, debug);
}

csi_fb_processing_impl::csi_fb_processing_impl(
    int num_ue, int rx_modtype, int nss, int ntx_gnb, bool wideband, int nrb, int pmi_bits, int cqi_bits,
    int crc_bits, int total_csi_bits, float polar_code_rate, bool debug)
    : gr::tagged_stream_block(
          "csi_fb_processing",
          gr::io_signature::make(num_ue, num_ue, sizeof(uint8_t)),
          gr::io_signature::make(0, 0, 0), "packet_len"),
      d_wideband(wideband), d_debug(debug),d_total_frames(0)
  {

      if (!d_wideband){
        throw std::runtime_error("[csi_fb_processing] Only wideband feedback implemented");
       }

      if (nss < 1 || nss > 2)
          throw std::runtime_error("only support 1 to 2 streams");
      d_nss = nss;
      if (ntx_gnb != 4)
          throw std::runtime_error("only support 4 gNB antennas supported for CSI Feedback");
      d_ntx_gnb = ntx_gnb;
      if (nss > ntx_gnb)
          throw std::runtime_error("number of streams must not larger than number of TX antennas");

      d_logfreq = 50;

      d_num_ue = num_ue;
      d_rx_modtype = rx_modtype;

      d_nrb = nrb;
      d_pmi_bits = pmi_bits;
      d_cqi_bits = cqi_bits;
      d_crc_bits = crc_bits;
      d_total_csi_bits = total_csi_bits;
      d_polar_code_rate = polar_code_rate;
      d_wb_cqi.assign(d_nss, 0);
      d_wb_cqi_db.assign(d_nss, 0.0f);
      build_codebook();

      message_port_register_out(pmt::mp("pmi"));
      message_port_register_out(pmt::mp("cqi"));

      set_tag_propagation_policy(block::TPP_DONT);

  }

csi_fb_processing_impl::~csi_fb_processing_impl() {}

int csi_fb_processing_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items) {
  int noutput_items = 0;
  return noutput_items;
}

int csi_fb_processing_impl::work(int noutput_items, gr_vector_int &ninput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items) {

    gr::thread::scoped_lock lock(fp_mutex);

    d_total_frames += 1;

    int min_inputs = ninput_items[0];
    for (int ch = 1; ch < d_num_ue; ++ch) {
        if (ninput_items[ch] < min_inputs)
            min_inputs = ninput_items[ch];
    }
    
    if (min_inputs < 1)
        return 0;

    for (int ch = 0; ch < d_num_ue; ++ch) {

        int nbytes = ninput_items[ch];

        if (nbytes * d_rx_modtype < d_total_csi_bits){
            dout << "[CSI FB Processing] incorrect number of bytes input" << std::endl;
            return 0;
        }

        const uint8_t* in = reinterpret_cast<const uint8_t*>(input_items[ch]);

        std::vector<uint8_t> bits(nbytes * d_rx_modtype);
        for (int i = 0; i < nbytes; i++) {
            uint8_t val = in[i];
            for (int b = 0; b < d_rx_modtype; b++) {
                bits[i * d_rx_modtype + b] = (val >> (d_rx_modtype - 1 - b)) & 1;
            }
        }

        bits.resize(d_total_csi_bits);

        /* ----------------------------------------------------------------
        * 1.  Polar decoding
        * ----------------------------------------------------------------*/

        std::vector<float> llr(bits.size());
        for (size_t i = 0; i < bits.size(); i++)
            llr[i] = bits[i] ? 10.0f : -10.0f;

        init_polar();
        int info_bits = static_cast<int>(std::round(d_total_csi_bits * d_polar_code_rate));
        std::vector<uint8_t> decoded(info_bits, 0);
        d_polar_dec->generic_work(llr.data(), decoded.data());

        if (info_bits < d_pmi_bits + d_cqi_bits + d_crc_bits)
            throw std::runtime_error("decoded CSI bits too short");

        /* ----------------------------------------------------------------
        * 2.  CRC Check
        * ----------------------------------------------------------------*/

        std::vector<uint8_t> relevant(decoded.begin(), decoded.begin() + d_pmi_bits + d_cqi_bits);
        uint16_t calc_crc = compute_crc10(relevant);
        uint16_t recv_crc = 0;
        for (int i = 0; i < d_crc_bits; i++)
            recv_crc |= (decoded[d_pmi_bits + d_cqi_bits + i] & 1) << i;

        if (calc_crc != recv_crc)
        {
            dout << "CRC Check failed" << std::endl;

            return 0;
        }
        else{
            if (d_total_frames % d_logfreq == 0)
                dout << "CRC Check passed" << std::endl;
        }

        /* ----------------------------------------------------------------
        * 3.  Recover codebook index from PMI
        * ----------------------------------------------------------------*/

        int pmi_idx = 0;
        if (d_total_frames % d_logfreq == 0)
            for (int i = 0; i < d_pmi_bits; i++)
            {
                pmi_idx |= (decoded[i] & 1) << i;
            }

        if (pmi_idx >= (int) d_codebook.size())
            throw std::runtime_error("invalid PMI index");

        if (d_total_frames % d_logfreq == 0)
            dout << "UE [" << ch << "] " << "received pmi_idx: " << pmi_idx << std::endl;

        CMatrixX Wm(d_ntx_gnb, d_nss);
        const auto &entry = d_codebook[pmi_idx];
        for (int tx = 0; tx < d_ntx_gnb; tx++)
            for (int s = 0; s < d_nss; s++)
                Wm(tx, s) = entry[tx * d_nss + s];

        CMatrixX reconstructed_H_mat = Wm.conjugate().transpose();


        /* ----------------------------------------------------------------
        * 4.  Recover per-stream SINR from CQI
        * ----------------------------------------------------------------*/

        d_wb_cqi.assign(d_nss, 0);
        d_wb_cqi_db.assign(d_nss, 0.0f);
        for (int s = 0; s < d_nss; ++s)
            for (int i = 0; i < 4 && (d_pmi_bits + 4 * s + i) < info_bits; ++i)
                d_wb_cqi[s] |= (decoded[d_pmi_bits + 4 * s + i] & 1) << i;

        constexpr float th_db[16] = {
            -1e3,-6.7,-4.7,-2.3,0.0,2.4,4.3,5.9,
            8.7,10.3,11.7,13.1,14.3,15.8,17.3,18.7};

        for (int s = 0; s < d_nss; ++s) {
            int idx = std::min<int>(std::max<int>(d_wb_cqi[s], 0), 15);
            d_wb_cqi_db[s] = th_db[idx];
            if (d_total_frames % d_logfreq == 0){
                dout << "UE [" << ch << "] " << "decoded CQI[" << s << "] = "
                    << int(d_wb_cqi[s]) << " -> " << d_wb_cqi_db[s]
                    << " dB" << std::endl;
            }
        }

        pmt::pmt_t pmi_meta = pmt::make_dict();
        pmi_meta = pmt::dict_add(pmi_meta, pmt::mp("pmi"), pmt::PMT_T);
        pmt::pmt_t pmi_blob = pmt::make_blob(reconstructed_H_mat.data(),
                                            sizeof(gr_complex) *
                                                reconstructed_H_mat.size());
        pmi_meta = pmt::dict_add(pmi_meta, pmt::mp("ue_idx"), pmt::from_long(ch));                                                
        message_port_pub(pmt::mp("pmi"), pmt::cons(pmi_meta, pmi_blob));

        pmt::pmt_t cqi_meta = pmt::make_dict();
        cqi_meta = pmt::dict_add(cqi_meta, pmt::mp("cqi"), pmt::PMT_T);
        pmt::pmt_t cqi_blob = pmt::make_blob(d_wb_cqi_db.data(),
                                            sizeof(float) * d_wb_cqi_db.size());
        cqi_meta = pmt::dict_add(cqi_meta, pmt::mp("ue_idx"), pmt::from_long(ch));                                            
        message_port_pub(pmt::mp("cqi"), pmt::cons(cqi_meta, cqi_blob));

    }

    return 0;
}

void csi_fb_processing_impl::build_codebook()
{
    constexpr int N1 = 2;
    constexpr int O1 = 4;
    constexpr int P_CSI_RS = 4;

    d_codebook.clear();

    if (d_ntx_gnb != 4)
        return;

    if (d_nss == 1) {
        const float inv_norm = 1.0f / std::sqrt(float(P_CSI_RS));
        d_codebook.reserve(N1 * O1 * 4);
        for (int l = 0; l < N1 * O1; ++l) {
            gr_complex v0{1.0f, 0.0f};
            float phi = float(M_PI * l) / 4.0f;
            gr_complex v1{std::cos(phi), std::sin(phi)};
            for (int n = 0; n < 4; ++n) {
                gr_complex phi_n;
                switch (n) {
                    case 0: phi_n = {1.0f, 0.0f}; break;
                    case 1: phi_n = {0.0f, 1.0f}; break;
                    case 2: phi_n = {-1.0f,0.0f}; break;
                    default: phi_n = {0.0f,-1.0f}; break;
                }
                std::array<gr_complex,8> w{};
                w[0] = inv_norm * v0;
                w[1] = inv_norm * v1;
                w[2] = inv_norm * phi_n * v0;
                w[3] = inv_norm * phi_n * v1;
                d_codebook.push_back(w);
            }
        }
        return;
    }

    if (d_nss == 2) {
        const float inv_norm = 1.0f / std::sqrt(2.0f * P_CSI_RS);
        d_codebook.reserve(N1 * O1 * 2 * 2);
        for (int l = 0; l < N1 * O1; ++l) {
            gr_complex v0{1.0f, 0.0f};
            float phi_l = float(M_PI * l) / 4.0f;
            gr_complex v1{std::cos(phi_l), std::sin(phi_l)};
            for (int i13 = 0; i13 < 2; ++i13) {
                int l2 = l + (i13 ? O1 : 0);
                float phi_l2 = float(M_PI * l2) / 4.0f;
                gr_complex v2{1.0f, 0.0f};
                gr_complex v3{std::cos(phi_l2), std::sin(phi_l2)};
                for (int n = 0; n < 2; ++n) {
                    gr_complex phi_n = (n == 0) ? gr_complex{1.0f,0.0f}
                                                : gr_complex{0.0f,1.0f};
                    std::array<gr_complex,8> w = {
                        inv_norm * v0,
                        inv_norm * v2,
                        inv_norm * v1,
                        inv_norm * v3,
                        inv_norm * phi_n * v0,
                        inv_norm * (-phi_n) * v2,
                        inv_norm * phi_n * v1,
                        inv_norm * (-phi_n) * v3};
                    d_codebook.push_back(w);
                }
            }
        }
    }
}

void csi_fb_processing_impl::init_polar()
{
    if (d_polar_inited)
        return;

    int block_size = d_total_csi_bits;
    int num_info_bits = static_cast<int>(std::round(d_total_csi_bits * d_polar_code_rate));
    int num_frozen_bits = block_size - num_info_bits;

    std::vector<int> frozen_positions(num_frozen_bits);
    for (int i = 0; i < num_frozen_bits && i < 128; i++)
        frozen_positions[i] = REL_SEQ_128[i];
    std::sort(frozen_positions.begin(), frozen_positions.end());
    std::vector<uint8_t> frozen_vals(num_frozen_bits, 0);

    d_polar_dec = std::static_pointer_cast<gr::fec::code::polar_decoder_sc>(
        gr::fec::code::polar_decoder_sc::make(
            block_size,
            num_info_bits,
            frozen_positions,
            frozen_vals));

    d_polar_inited = true;
}

uint16_t csi_fb_processing_impl::compute_crc10(const std::vector<uint8_t> &bits) const
{
    uint16_t crc = 0;
    for (size_t i = 0; i < bits.size(); i++)
    {
        bool in_bit = (bits[bits.size() - 1 - i] != 0);
        bool top_bit = ((crc & 0x200) != 0);
        bool feedback = (in_bit ^ top_bit);
        crc <<= 1;
        crc &= 0x3FF;
        if (feedback)
            crc ^= 0x233;
    }
    return (crc & 0x3FF);
}

} /* namespace ncjt */
} /* namespace gr */
