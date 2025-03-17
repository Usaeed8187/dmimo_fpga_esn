/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_SIC_DETECT_IMPL_H
#define INCLUDED_NCJT_SIC_DETECT_IMPL_H

#include <gnuradio/ncjt/sic_detect.h>

namespace gr::ncjt
{

class sic_detect_impl : public sic_detect
{
private:
    const int MAX_NSS = 8; // maximum number of streams/antennas supported
    int d_nss; // number of spatial streams
    int d_nrx; // number of receive antennas
    int d_num_symbols; // number of symbols per frame/packet
    int d_cur_symbol; // current OFDM symbol index
    int d_total_pkts; // total number of packets received

    int d_scnum; // number of valid subcarriers
    int d_scdata; // number data subcarriers
    int d_modtype; // modulation order (2-QPSK, 4-16QAM, 6-64QAm, 8-256QAM)
    std::vector<gr_complex> QAM_CONST;

    float d_cpe_phi1, d_cpe_phi2; // CPE estimation slope
    float d_cpe_offset1, d_cpe_offset2; // CPE estimation offset

    gr_complex *d_chan_est_buf; // channel estimation using H-LTFs
    gr_complex *d_mmse_weight; // MMSE channel equalization coefficients
    gr_complex *d_chan_coef; // channel coefficients for canceling streams
    int *d_sic_order; // SIC processing order for all subcarriers
    bool d_chan_est_ready;

    const bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    update_mmse_sic_coef(float nvar);

    gr_complex
    qam_detect(const gr_complex yh);

    void
    add_packet_tag(uint64_t offset, int packet_len);

public:
    sic_detect_impl(int fftsize, int nrx, int nss, int modtype, int ndatasymbols, bool debug);
    ~sic_detect_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SIC_DETECT_IMPL_H */
