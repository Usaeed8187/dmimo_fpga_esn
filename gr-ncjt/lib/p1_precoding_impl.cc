/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "p1_precoding_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <Eigen/Dense>
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Eigenvalues>


namespace gr {
namespace ncjt {

p1_precoding::sptr p1_precoding::make(int num_ue, int nss, int ntx_gnb, int num_iter, int numltfsyms,
                                      int numdatasyms,
                                      bool wideband, int nfft, int num_sc, int logfreq,
                                      bool debug) {
  return gnuradio::make_block_sptr<p1_precoding_impl>(
      num_ue, nss, ntx_gnb, num_iter, numltfsyms, numdatasyms, wideband, nfft,
      num_sc, logfreq, debug);
}

p1_precoding_impl::p1_precoding_impl(int num_ue, int nss, int ntx_gnb, int num_iter, int numltfsyms,
                                     int numdatasyms,
                                     bool wideband, int nfft, int num_sc, int logfreq,
                                     bool debug)
    : gr::tagged_stream_block(
          "p1_precoding",
          gr::io_signature::make(nss, nss, sizeof(gr_complex)),
          gr::io_signature::make(ntx_gnb, ntx_gnb, sizeof(gr_complex)), "packet_len"),
      d_wideband(wideband), d_debug(debug)
{

    if (nss != 1 && nss != 2)
        throw std::runtime_error("only support 1 to 2 streams");
    d_nss = nss;
    if (ntx_gnb != 2 && ntx_gnb != 4)
        throw std::runtime_error("only support 2/4 gNB antennas");
    d_ntx_gnb = ntx_gnb;
    if (nss > ntx_gnb)
        throw std::runtime_error("number of streams must not larger than number of TX antennas");

    d_logfreq = logfreq;

    d_num_iter = num_iter;

    d_num_ue = num_ue;

    d_pmi_ue.assign(d_num_ue, CMatrixX::Zero(d_nss, d_ntx_gnb));  // sets size & value
    d_cqi_ue.resize(d_num_ue);                                    // empty inner vectors

    d_num_symbols = numltfsyms + numdatasyms;
    d_nfft = nfft;
    d_num_sc = num_sc;

    // allocate d_map_matrix
    size_t map_size = d_ntx_gnb * d_nss * d_num_sc * sizeof(gr_complex);
    d_map_matrix = (gr_complex *) volk_malloc(map_size, volk_get_alignment());
    if (!d_map_matrix)
        throw std::runtime_error("Failed to allocate d_map_matrix");

    // Default mapping matrix using CSD (row-major layout)
    int cshift[] = {0, -8, -4, -12, -7, -13}; // cyclic shift
    for (int k = 0; k < d_num_sc; k++)
        for (int m = 0; m < d_ntx_gnb; m++)
            for (int n = 0; n < d_nss; n++)
            {
                int ci = (k < d_num_sc/2) ? (k - d_num_sc/2) : (k - d_num_sc/2 + 1);
                int si = m / d_nss;
                gr_complex cs = exp(gr_complex (0 ,-2.0*M_PI*cshift[si]*ci/double(d_nfft)));
                d_map_matrix[k * d_ntx_gnb * d_nss + d_nss * m + n] = (m % d_nss == n) ? cs : 0.0;
            }

    // Initial loading of offline CSI data
    // Note: CSI data should use row-major layout

    message_port_register_in(pmt::mp("pmi"));
    message_port_register_in(pmt::mp("cqi"));
    set_msg_handler(pmt::mp("pmi"), [this](const pmt::pmt_t &msg) { process_pmi_message(msg); });
    set_msg_handler(pmt::mp("cqi"), [this](const pmt::pmt_t &msg) { process_cqi_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);

}

p1_precoding_impl::~p1_precoding_impl() {
    if (d_map_matrix != nullptr)
        volk_free(d_map_matrix);
}

int p1_precoding_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = d_num_symbols * d_num_sc;
    return noutput_items;
}

int p1_precoding_impl::work(int noutput_items, gr_vector_int &ninput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items) {

    gr::thread::scoped_lock lock(fp_mutex);

    d_total_frames += 1;

    // apply precoding or spatial mapping
    if (d_nss == 2 && d_ntx_gnb == 2)
        apply_mapping_2tx(input_items, output_items);
    else if (d_nss == 4 && d_ntx_gnb == 4)
        apply_mapping_4tx(input_items, output_items);
    else
        apply_mapping_ntx(input_items, output_items);

    return d_num_symbols * d_num_sc;
}

void
p1_precoding_impl::apply_mapping_2tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto in1 = (const gr_complex *) input_items[1];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    for (int n = 0; n < d_num_symbols; n++)
    {
        CMatrixX Y(d_ntx_gnb, 1);  // (Nt, 1)
        for (int k = 0; k < d_num_sc; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            X(0, 0) = in0[n * d_num_sc + k];
            X(1, 0) = in1[n * d_num_sc + k];
            Eigen::Map<CMatrixX> Q(&d_map_matrix[d_nss * d_ntx_gnb * k], d_ntx_gnb, d_nss); // (Nt, Nss)
            Y = Q * X;
            out0[n * d_num_sc + k] = Y(0, 0);
            out1[n * d_num_sc + k] = Y(1, 0);
        }
    }
}

void
p1_precoding_impl::apply_mapping_4tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto in1 = (const gr_complex *) input_items[1];
    auto in2 = (const gr_complex *) input_items[2];
    auto in3 = (const gr_complex *) input_items[3];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];
    auto out2 = (gr_complex *) output_items[2];
    auto out3 = (gr_complex *) output_items[3];

    for (int n = 0; n < d_num_symbols; n++)
    {
        CMatrixX Y(d_ntx_gnb, 1);  // (Nt, 1)
        for (int k = 0; k < d_num_sc; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            X(0, 0) = in0[n * d_num_sc + k];
            X(1, 0) = in1[n * d_num_sc + k];
            X(2, 0) = in2[n * d_num_sc + k];
            X(3, 0) = in3[n * d_num_sc + k];
            Eigen::Map<CMatrixX> Q(&d_map_matrix[d_nss * d_ntx_gnb * k], d_ntx_gnb, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            out0[n * d_num_sc + k] = Y(0, 0);
            out1[n * d_num_sc + k] = Y(1, 0);
            out2[n * d_num_sc + k] = Y(2, 0);
            out3[n * d_num_sc + k] = Y(3, 0);
        }
    }
}

void
p1_precoding_impl::apply_mapping_ntx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    for (int n = 0; n < d_num_symbols; n++)
    {
        int mtx_size = d_nss * d_ntx_gnb;
        CMatrixX Y(d_ntx_gnb, 1);  // (Nt, 1)
        for (int k = 0; k < d_num_sc; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            for (int s = 0; s < d_nss; s++)
            {
                auto in = (const gr_complex *) input_items[s];
                X(s, 0) = in[n * d_num_sc + k];
            }
            Eigen::Map<CMatrixX> Q(&d_map_matrix[mtx_size * k], d_ntx_gnb, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            for (int m = 0; m < d_ntx_gnb; m++)
            {
                auto out = (gr_complex *) output_items[m];
                out[n * d_num_sc + k] = Y(m, 0);
            }
        }
    }
}

void
p1_precoding_impl::process_pmi_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t blob = pmt::cdr(msg);

    if (!pmt::dict_has_key(meta, pmt::mp("pmi")))
        throw std::runtime_error("invalid PMI message");
    
    /* UE/stream index (defaults to 0 if the tag is absent) -------------- */
    int ue = 0;
    if (pmt::dict_has_key(meta, pmt::mp("ue_idx")))
        ue = pmt::to_long(pmt::dict_ref(meta, pmt::mp("ue_idx"),
                                        pmt::from_long(0)));
    if (ue < 0 || ue >= d_num_ue)
        throw std::runtime_error("invalid UE index in PMI message");
    
    /* blob → Eigen matrix ----------------------------------------------- */
    const size_t nbytes  = pmt::blob_length(blob);
    const size_t nelems  = nbytes / sizeof(gr_complex);
    if (nelems != static_cast<size_t>(d_nss * d_ntx_gnb))
        throw std::runtime_error("unexpected PMI matrix size");

    const gr_complex *data = static_cast<const gr_complex*>(pmt::blob_data(blob));
    d_pmi_ue[ue] = Eigen::Map<const CMatrixX>(data, d_nss, d_ntx_gnb);

    // update precoder using last PMI/CQI information
    update_p1_weighted_mean_precoding();
}

void
p1_precoding_impl::process_cqi_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t blob = pmt::cdr(msg);

    if (!pmt::dict_has_key(meta, pmt::mp("cqi")))
        throw std::runtime_error("invalid CQI message (no tag)");

    int ue = 0;
    if (pmt::dict_has_key(meta, pmt::mp("ue_idx")))
        ue = pmt::to_long(pmt::dict_ref(meta, pmt::mp("ue_idx"),
                                        pmt::from_long(0)));
    if (ue < 0 || ue >= d_num_ue)
        throw std::runtime_error("invalid UE index in CQI message");

    const size_t nbytes = pmt::blob_length(blob);
    const size_t nvals  = nbytes / sizeof(float);

    d_cqi_ue[ue].resize(nvals);
    std::memcpy(d_cqi_ue[ue].data(), pmt::blob_data(blob), nbytes);

}

void
p1_precoding_impl::update_p1_weighted_mean_precoding()
{
    /* ---------- basic sanity checks ---------------------------------- */
    if (d_pmi_ue.empty() || d_pmi_ue[0].size() == 0)
        return;                                 // no CSI yet

    const int Nt          = d_ntx_gnb;         // gNB TX antennas

    /* ---------- algorithm constants ---------------------------------- */
    const float weight_floor_constant = 0.10f;
    const float gram_constant        = 0.01f;   // ε·I regulariser
    const float n_var                = 1e-3f;   // flat noise var (TODO: derive from CQI/SNR)

    /* ---------- state containers ------------------------------------- */
    Eigen::MatrixXf w_k_i          = Eigen::MatrixXf::Ones(d_num_ue, d_nss);
    Eigen::MatrixXf best_w_k_i     = w_k_i;
    Eigen::MatrixXf SINR_k_i       = Eigen::MatrixXf::Zero(d_num_ue, d_nss);
    Eigen::MatrixXf best_SINR_k_i  = SINR_k_i;          // keeps best min-SINR snapshot

    /* ---------- main iterative loop ---------------------------------- */
    for (int iter = 0; iter <= d_num_iter; ++iter)
    {
        /* ---- build ∑_k (H_k √w_k)ᴴ (H_k √w_k) ----------------------- */
        CMatrixX sumGram = CMatrixX::Zero(Nt, Nt);

        for (int k = 0; k < d_num_ue; ++k)
        {
            const CMatrixX& H = d_pmi_ue[k];            // shape (Nss × Nt)
            if (H.rows() == 0) continue;                // skip if no CSI yet

            CMatrixX H_weighted = H;                    // row-scale by √w
            for (int i = 0; i < d_nss; ++i)
                H_weighted.row(i) *= std::sqrt(w_k_i(k, i));

            sumGram.noalias() += H_weighted.adjoint() * H_weighted;
        }
        sumGram += gram_constant * CMatrixX::Identity(Nt, Nt);

        /* ---- eigen-decomp – keep strongest Nss eigenvectors --------- */
        Eigen::SelfAdjointEigenSolver<CMatrixX> es(sumGram);
        CMatrixX V = es.eigenvectors().rightCols(d_nss);   // Nt × Nss

        /* ---- per-UE/SINR evaluation -------------------------------- */
        for (int k = 0; k < d_num_ue; ++k)
        {
            const CMatrixX& H = d_pmi_ue[k];
            if (H.rows() == 0) { SINR_k_i.row(k).setZero(); continue; }

            CMatrixX Hkv = H * V;                   // Nss × Nss

            if (d_nss == 2) {
                for (int i = 0; i < d_nss; ++i) {
                    float signal = Hkv.col(i).squaredNorm();
                    float interf  = 0.f;
                    for (int j = 0; j < d_nss; ++j)
                        if (j != i)  interf += Hkv.col(j).squaredNorm();
                    SINR_k_i(k, i) = signal / (interf + n_var);
                }
            } else {
                SINR_k_i(k,0) = Hkv.squaredNorm() / n_var;
            }
        }

        /* ---- keep the weight set that maximises min-SINR ------------ */
        if (iter == 0 || SINR_k_i.minCoeff() > best_SINR_k_i.minCoeff()) {
            best_w_k_i    = w_k_i;
            best_SINR_k_i = SINR_k_i;
        }

        /* ---- update weights for next iteration (skip after last) ---- */
        if (iter < d_num_iter) {
            w_k_i = SINR_k_i.array().inverse().sqrt();

            const float floor_val = weight_floor_constant * w_k_i.mean();
            for (int r = 0; r < w_k_i.rows(); ++r)
                for (int c = 0; c < w_k_i.cols(); ++c)
                    if (w_k_i(r,c) < floor_val) w_k_i(r,c) = floor_val;
            /* (no renorm – matches python, which leaves ∑w unconstrained) */
        }
    }

    /* ---------- final precoder with best w_k_i ----------------------- */
    CMatrixX sumGram = CMatrixX::Zero(Nt, Nt);
    for (int k = 0; k < d_num_ue; ++k)
    {
        const CMatrixX& H = d_pmi_ue[k];
        if (H.rows() == 0) continue;

        CMatrixX H_weighted = H;
        for (int i = 0; i < d_nss; ++i)
            H_weighted.row(i) *= std::sqrt(best_w_k_i(k, i));

        sumGram.noalias() += H_weighted.adjoint() * H_weighted;
    }
    sumGram += gram_constant * CMatrixX::Identity(Nt, Nt);

    Eigen::SelfAdjointEigenSolver<CMatrixX> es(sumGram);
    CMatrixX V_final = es.eigenvectors().rightCols(d_nss);

    /* ---- column-wise power normalisation --------------------------- */
    for (int i = 0; i < d_nss; ++i)
        V_final.col(i).normalize();

    /* ---------- write into d_map_matrix (all sub-carriers) ---------- */
    for (int k = 0; k < d_num_sc; ++k) {
        Eigen::Map<CMatrixX> W(&d_map_matrix[Nt * d_nss * k], Nt, d_nss);
        W = V_final;
    }
}


} /* namespace ncjt */
} /* namespace gr */
