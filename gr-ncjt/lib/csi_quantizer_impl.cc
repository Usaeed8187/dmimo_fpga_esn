/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 */

#include "csi_quantizer_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

namespace gr::ncjt {

const int csi_quantizer_impl::REL_SEQ_128[128] = {
    33,34,35,36,37,38,40,41,42,44,48,49,76,52,56,0,
    65,66,67,68,69,70,72,73,74,50,1,2,3,4,5,6,
    7,8,9,10,11,12,32,14,16,17,18,19,20,21,22,24,
    25,26,28,13,64,112,96,80,81,82,104,84,88,97,100,98,
    15,23,27,29,39,30,43,45,46,51,71,53,75,54,77,57,
    83,78,58,85,86,60,89,99,90,101,102,92,105,106,108,113,
    114,116,120,31,47,55,79,59,87,61,91,62,103,93,107,94,
    109,115,110,117,118,121,122,124,63,95,111,119,123,125,126,127 };

csi_quantizer::sptr csi_quantizer::make(int fftsize,
                                 int ntx,
                                 int nrx,
                                 int nss,
                                 int rbg_size,
                                 int n_ofdm_syms_fb,
                                 int logfreq,
                                 bool debug)
{
    return gnuradio::make_block_sptr<csi_quantizer_impl>(fftsize,
                                                     ntx,
                                                     nrx,
                                                     nss,
                                                     rbg_size,
                                                     n_ofdm_syms_fb,
                                                     logfreq,
                                                     debug);
}

csi_quantizer_impl::csi_quantizer_impl(int fftsize,
                                       int ntx,
                                       int nrx,
                                       int nss,
                                       int rbg_size,
                                       int n_ofdm_syms_fb,
                                       int logfreq,
                                       bool debug)
    : gr::tagged_stream_block("csi_quantizer",
                              gr::io_signature::make(0, 0, 0),
                              gr::io_signature::make(1, 1, sizeof(uint8_t)),
                              "packet_len"),
      d_fftsize(fftsize), d_modtype(2), d_ntx(ntx), d_nss(nss), d_nrx(nrx),
      d_rbg_size(rbg_size), d_ofdm_syms_fb(n_ofdm_syms_fb), d_logfreq(logfreq),
      d_debug(debug)
{
    d_total_frames = 0;

    if (fftsize == 64) {
        d_scnum = 56;
        d_npt = 4;
    } else if (fftsize == 256) {
        d_scnum = 242;
        d_npt = 8;
    } else {
        throw std::runtime_error("Unsupported FFT size");
    }
    d_sdnum = d_scnum - d_npt;

    if (ntx != 4 || nss > 2)
        throw std::runtime_error("only support 4 tx antennas and up to 2 spatial streams");
    if (nrx < 1 || nrx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Rx antennas");
    if (nrx < nss)
        throw std::runtime_error("number of Rx antennas must be >= streams");
    if (rbg_size <= 0)
        throw std::runtime_error("rbg_size must be > 0");

    d_chan_csi.resize(d_ntx * d_nrx * d_scnum, gr_complex(0,0));
    d_wb_cqi.resize(d_nss, 0);

    build_codebook();
    d_exp_bits = d_ofdm_syms_fb * d_sdnum * d_modtype;
    std::mt19937                 gen{std::random_device{}()};
    std::uniform_int_distribution<int> bit{0,1};
    d_out_buff.resize(d_exp_bits);
    std::generate(d_out_buff.begin(), d_out_buff.end(),
                  [&]{ return static_cast<uint8_t>(bit(gen)); });

    std::stringstream str; str << name() << unique_id();
    d_id = pmt::string_to_symbol(str.str());

    message_port_register_in(pmt::mp("csi"));
    set_msg_handler(pmt::mp("csi"), [this](const pmt::pmt_t& msg) { process_csi_message(msg); });

    message_port_register_out(pmt::mp("csi_bits"));

    set_tag_propagation_policy(TPP_DONT);    
}

csi_quantizer_impl::~csi_quantizer_impl() {}


int
csi_quantizer_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    // size_t noutput_bytes = (d_exp_bits + 7) >> 3; // round-up division by 8
    return d_exp_bits;
}


int
csi_quantizer_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    // if (d_total_frames % d_logfreq == 0)
    //   dout << "in work" << std::endl;
    std::lock_guard<std::mutex> guard(d_mutex);

    // size_t noutput_bytes = (d_exp_bits + 7) >> 3; // round-up division by 8
    
    if (static_cast<size_t>(noutput_items) < static_cast<size_t>(d_exp_bits)){
      // if (d_total_frames % d_logfreq == 0)
      //   dout << "too few noutput_items" << std::endl;
      return 0;
    }

    auto *out = (uint8_t*)output_items[0];

    // if (d_total_frames % d_logfreq == 0){
    //   dout << "trying to feed out this many bytes: " << sizeof(uint8_t) * noutput_bytes << std::endl;
    // }

    // std::memcpy(&out[0], d_out_buff.data(), sizeof(uint8_t) * noutput_bytes);
    std::copy(d_out_buff.begin(), d_out_buff.end(), out);

    // if (d_total_frames % d_logfreq == 0){
    //   dout << "successfully fed out bytes" << std::endl;
    // }

    uint64_t offset = nitems_written(0);
    // add_packet_tag(offset, noutput_bytes);
    add_packet_tag(offset, d_exp_bits);


    // if (d_total_frames % d_logfreq == 0){
    //   dout << "added packet tag" << std::endl;
    // }

    // if (d_total_frames % d_logfreq == 0){
    //   dout << "returning from work" << std::endl;
    // }

    return d_exp_bits;
}


void
csi_quantizer_impl::add_packet_tag(uint64_t offset, int packet_len)
{
  add_item_tag(0, offset,
                pmt::string_to_symbol("packet_len"),
                pmt::from_long(packet_len), d_id);
}

void csi_quantizer_impl::process_csi_message(const pmt::pmt_t &msg)
{   
    // if (d_total_frames % d_logfreq == 0)
    //   dout << "in process_csi_message" << std::endl;
    std::lock_guard<std::mutex> guard(d_mutex);
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t blob = pmt::cdr(msg);

    if (pmt::dict_has_key(meta, pmt::mp("reset")))
        return;

    size_t nbytes = pmt::blob_length(blob);
    if (nbytes != d_chan_csi.size() * sizeof(gr_complex))
        throw std::runtime_error("invalid CSI blob length");

    const gr_complex* data = (const gr_complex*)pmt::blob_data(blob);
    std::copy(data, data + d_chan_csi.size(), d_chan_csi.begin());

    compute_quantised_csi();

    /* Build bit sequence */
    std::vector<uint8_t> bits;
    bits.reserve(64);
    for (int i=0;i<5;i++)
        bits.push_back((d_wb_pmi_idx >> i) & 1);
    for (auto cqi_idx : d_wb_cqi)
        for (int i=0;i<4;i++)
            bits.push_back((cqi_idx >> i) & 1);
    uint16_t crc = compute_crc10(bits);
    for (int i=0;i<10;i++)
        bits.push_back((crc >> i) & 1);
    bits.resize(64,0);

    init_polar();
    std::vector<uint8_t> coded(128,0);
    d_polar_enc->generic_work(bits.data(), coded.data());

    // if (d_total_frames % d_logfreq == 0){
    //   dout << "down-stream block expects this many bits: " << d_exp_bits << std::endl;
    //   dout << "we are tring to feed back this many bits: " << coded.size() << std::endl;
    // }

    if (d_exp_bits > static_cast<int>(coded.size())){
//      coded.resize(d_exp_bits, 0);
      static thread_local std::mt19937 gen{std::random_device{}()};
      std::uniform_int_distribution<int> bit_dist(0, 1);
      coded.reserve(d_exp_bits);

      while (static_cast<int>(coded.size()) < d_exp_bits) {
          coded.push_back(static_cast<uint8_t>(bit_dist(gen)));
      }

        // if (d_total_frames % d_logfreq == 0){
      //   dout << "updated the number of bits we are trying to feed back to: " << coded.size() << std::endl;
      // }
    }
    else if (d_exp_bits < static_cast<int>(coded.size()))
      throw std::runtime_error("size of feedback too large for configured parameters: consider increasing number of OFDM symbols for feedback");

    // d_out_buff.assign((coded.size()+7)/8, 0);
    // for (size_t i=0;i<coded.size();++i)
    //     d_out_buff[i>>3] |= (coded[i] & 1) << (i & 7);
    d_out_buff = coded;

    // if (d_total_frames % d_logfreq == 0){
    //   dout << "size of d_out_buff = " << d_out_buff.size() << std::endl;
    // }

    d_add_tag = true;
}

void csi_quantizer_impl::init_polar()
{
    if (d_polar_inited)
        return;

    int block_size = 128;
    int num_info_bits = 64;
    int num_frozen_bits = block_size - num_info_bits;

    std::vector<int> frozen_positions(num_frozen_bits);
    for (int i=0;i<num_frozen_bits;i++)
        frozen_positions[i] = REL_SEQ_128[i];
    std::sort(frozen_positions.begin(), frozen_positions.end());
    std::vector<uint8_t> frozen_vals(num_frozen_bits,0);

    d_polar_enc = std::static_pointer_cast<gr::fec::code::polar_encoder>(
        gr::fec::code::polar_encoder::make(block_size,
                                           num_info_bits,
                                           frozen_positions,
                                           frozen_vals,
                                           false));
    d_polar_inited = true;
}

uint16_t csi_quantizer_impl::compute_crc10(const std::vector<uint8_t>& bits) const
{
    uint16_t crc = 0;
    for (size_t i=0;i<bits.size();i++) {
        bool in_bit = (bits[bits.size()-1-i] != 0);
        bool top_bit = ((crc & 0x200) != 0);
        bool feedback = (in_bit ^ top_bit);
        crc <<= 1; crc &= 0x3FF;
        if (feedback)
            crc ^= 0x233;
    }
    return (crc & 0x3FF);
}

void csi_quantizer_impl::build_codebook()
{
    constexpr int  N1 = 2;
    constexpr int  O1 = 4;
    constexpr int  P_CSI_RS = 4;

    d_codebook.clear();

    if (d_nss == 1) {
        const float inv_norm = 1.0f / std::sqrt(float(P_CSI_RS));
        d_codebook.reserve(N1*O1*4);
        for (int l=0;l<N1*O1;++l) {
            gr_complex v0{1.0f,0.0f};
            float phi = float(M_PI*l)/4.0f;
            gr_complex v1{std::cos(phi), std::sin(phi)};
            for (int n=0;n<4;++n) {
                gr_complex phi_n;
                switch(n){
                    case 0: phi_n={1.0f,0.0f}; break;
                    case 1: phi_n={0.0f,1.0f}; break;
                    case 2: phi_n={-1.0f,0.0f}; break;
                    default:phi_n={0.0f,-1.0f}; break;
                }
                std::array<gr_complex,8> w{};
                w[0]=inv_norm*v0; w[1]=inv_norm*v1;
                w[2]=inv_norm*phi_n*v0; w[3]=inv_norm*phi_n*v1;
                d_codebook.push_back(w);
            }
        }
        return;
    }

    if (d_nss == 2) {
        const float inv_norm = 1.0f / std::sqrt(2.0f * P_CSI_RS);
        d_codebook.reserve(N1*O1*2*2);
        for (int l=0;l<N1*O1;++l) {
            gr_complex v0{1.0f,0.0f};
            float phi_l = float(M_PI*l)/4.0f;
            gr_complex v1{std::cos(phi_l), std::sin(phi_l)};
            for (int i13=0;i13<2;++i13) {
                int l2 = l + (i13 ? O1 : 0);
                float phi_l2 = float(M_PI*l2)/4.0f;
                gr_complex v2{1.0f,0.0f};
                gr_complex v3{std::cos(phi_l2), std::sin(phi_l2)};
                for (int n=0;n<2;++n) {
                    gr_complex phi_n = (n==0)?gr_complex{1.0f,0.0f}:gr_complex{0.0f,1.0f};
                    std::array<gr_complex,8> w = {
                        inv_norm*v0,
                        inv_norm*v2,
                        inv_norm*v1,
                        inv_norm*v3,
                        inv_norm*phi_n*v0,
                        inv_norm*(-phi_n)*v2,
                        inv_norm*phi_n*v1,
                        inv_norm*(-phi_n)*v3};
                    d_codebook.push_back(w);
                }
            }
        }
        return;
    }

    throw std::runtime_error("build_codebook(): unsupported number of streams");
}

void csi_quantizer_impl::compute_quantised_csi()
{
    if (d_codebook.empty()) {
        d_wb_pmi_idx = 0;
        return;
    }

    const int Nt = d_ntx;
    const int Ns = d_nss;
    const int Nr = d_nrx;
    const int Nsc = d_scnum;
    const int RBG = d_rbg_size;
    const int Ncb = static_cast<int>(d_codebook.size());

    auto H_idx = [=](int sc, int tx, int rx){ return Nt*Nr*sc + Nr*tx + rx; };

    std::vector<uint8_t> rbg_pmi; rbg_pmi.reserve((Nsc+RBG-1)/RBG);

    for (int sc0=0; sc0<Nsc; sc0+=RBG) {
        int sc1 = std::min(sc0+RBG, Nsc);
        gr_complex Havg[Nt][MAX_NSS]{};
        for (int tx=0; tx<Nt; ++tx)
            for (int rx=0; rx<Nr; ++rx) {
                gr_complex acc(0,0);
                for (int sc=sc0; sc<sc1; ++sc)
                    acc += d_chan_csi[H_idx(sc,tx,rx)];
                Havg[tx][rx] = acc / float(sc1 - sc0);
            }
        uint8_t best=0; float best_metric=-1.0f;
        for(uint8_t c=0;c<Ncb;++c){
            const auto &w=d_codebook[c];
            float metric=0.0f;
            for(int rx=0;rx<Nr;++rx){
                for(int s=0;s<Ns;++s){
                    gr_complex dot(0,0);
                    for(int tx=0;tx<Nt;++tx){
                        int widx=(Ns==1)?tx:tx*Ns+s;
                        dot += std::conj(Havg[tx][rx])*w[widx];
                    }
                    metric += std::norm(dot);
                }
            }
            if(metric>best_metric){best_metric=metric;best=c;}
        }
        rbg_pmi.push_back(best);
    }

    std::vector<int> hist(Ncb,0);
    for(auto idx:rbg_pmi) ++hist[idx];
    d_wb_pmi_idx = static_cast<uint8_t>(std::max_element(hist.begin(),hist.end())-hist.begin());

    std::vector<double> sinr_acc(Ns,0.0);
    int rbg_cnt=0;
    d_rbg_cqi.clear();
    constexpr float th_db[16] = {
          -1e3,-6.7,-4.7,-2.3,0.0,2.4,4.3,5.9,
          8.7,10.3,11.7,13.1,14.3,15.8,17.3,18.7};
    for(int sc0=0; sc0<Nsc; sc0+=RBG){
        int sc1 = std::min(sc0+RBG, Nsc);
        ++rbg_cnt;
        gr_complex Havg[Nt][MAX_NSS]{};
        for(int tx=0; tx<Nt; ++tx)
            for(int rx=0; rx<Nr; ++rx){
                gr_complex acc(0,0);
                for(int sc=sc0; sc<sc1; ++sc)
                    acc += d_chan_csi[H_idx(sc,tx,rx)];
                Havg[tx][rx] = acc / float(sc1-sc0);
            }
        const auto &entry = d_codebook[d_wb_pmi_idx];
        CMatrixX Wm(d_ntx, d_nss);
        for(int tx=0; tx<d_ntx; ++tx)
            for(int s=0; s<d_nss; ++s)
                Wm(tx,s)=entry[tx*d_nss+s];
        CMatrixX Ht(Nr,Nt);
        for(int tx=0; tx<d_ntx; ++tx)
            for(int rx=0; rx<d_nrx; ++rx)
                Ht(rx,tx)=Havg[tx][rx];
        CMatrixX Heff = Ht * Wm;
        CMatrixX Ha = Heff.adjoint();
        CMatrixX HH = Ha * Heff + d_noise_est * CMatrixX::Identity(d_nss,d_nss);
        CMatrixX G = HH.inverse() * Ha;
        CMatrixX S = G * Heff;
        std::vector<uint8_t> cqi_stream(d_nss,0);
        for(int s=0; s<d_nss; ++s){
            double num = std::norm(S(s,s));
            // if (d_total_frames % d_logfreq == 0){
            //   dout << "numerator when calculating SINR for stream " << s << ": " << num << std::endl;
            // }
            double den = 0.0;
            for(int j=0;j<d_nss;++j){ if(j!=s) den += std::norm(S(s,j)); }
            den += d_noise_est * G.row(s).squaredNorm();
            double sinr = (den>0.0)? num/den : 0.0;
            double sinr_db = 10.0*std::log10(sinr + 1e-12);
            int cqi=0; while(cqi<15 && sinr_db>th_db[cqi+1]) ++cqi;
            cqi_stream[s]=static_cast<uint8_t>(cqi);
            sinr_acc[s]+=sinr;
        }
        uint8_t cqi_rbg = *std::min_element(cqi_stream.begin(), cqi_stream.end());
        d_rbg_cqi.push_back(cqi_rbg);
    }
    d_wb_cqi.assign(Ns,0);
    for(int s=0; s<Ns; ++s){
        double mean_sinr = (rbg_cnt>0)? sinr_acc[s]/rbg_cnt : 10;
        // if (d_total_frames % d_logfreq == 0){
        //   dout << "mean_sinr for stream " << s << ": " << mean_sinr << std::endl;
        // }
        double sinr_db = 10.0*std::log10(mean_sinr + 1e-12);
        int cqi=0; while(cqi<15 && sinr_db>th_db[cqi+1]) ++cqi;
        d_wb_cqi[s]=static_cast<uint8_t>(cqi);
    }
    if (d_total_frames % d_logfreq == 0){
      dout << "PMI index: " << static_cast<int>(d_wb_pmi_idx) << std::endl;
      for(int s=0; s<Ns; ++s){
        auto cqi = static_cast<int>(d_wb_cqi[s]);
        dout << "CQI[" << s << "] index: " << cqi << " -> " << th_db[cqi] << " dB SINR" << std::endl;
      }
    }

    std::vector<uint8_t> fb_bits;
    fb_bits.reserve(13);
    for (int i = 0; i < 5; i++)
        fb_bits.push_back((d_wb_pmi_idx >> i) & 1);
    for (int s = 0; s < std::min(d_nss, 2); ++s)
        for (int i = 0; i < 4; i++)
            fb_bits.push_back((d_wb_cqi[s] >> i) & 1);
    while (fb_bits.size() < 13)
        fb_bits.push_back(0);
    pmt::pmt_t fb_meta = pmt::make_dict();
    fb_meta = pmt::dict_add(fb_meta, pmt::mp("csi_bits"), pmt::PMT_T);
    pmt::pmt_t fb_blob = pmt::make_blob(fb_bits.data(), fb_bits.size());
    message_port_pub(pmt::mp("csi_bits"), pmt::cons(fb_meta, fb_blob));

    d_total_frames += 1;

}

} /* namespace gr::ncjt */
