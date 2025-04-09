/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_MIMO_DETECT_IMPL_H
#define INCLUDED_NCJT_MIMO_DETECT_IMPL_H

#include <gnuradio/ncjt/mimo_detect.h>

namespace gr::ncjt
{

class mimo_detect_impl : public mimo_detect
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

    float d_cpe_phi1, d_cpe_phi2; // CPE estimation slope
    float d_cpe_offset1, d_cpe_offset2; // CPE estimation offset

    gr_complex *d_chan_est_buf; // channel estimation using H-LTFs
    gr_complex *d_mmse_coef; // MMSE channel equalization coefficients

    pmt::pmt_t _id;
    const int d_logfreq;
    const bool d_debug;

    void
    update_mmse_coef_2rx(float nvar);

    void
    update_mmse_coef_4rx(float nvar);

    void
    update_mmse_coef_nrx(float nvar);

    void
    check_cpe_tags(int ninput_syms);

    void
    add_packet_tag(uint64_t offset, int packet_len);

    void
    send_maxllr_message(float maxllr);

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

public:
    mimo_detect_impl(int fftsize, int nrx, int nss, int ndatasymbols, int logfreq, bool debug);
    ~mimo_detect_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MIMO_DETECT_IMPL_H */
