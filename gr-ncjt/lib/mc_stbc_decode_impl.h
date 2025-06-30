/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_MC_STBC_DECODE_IMPL_H
#define INCLUDED_NCJT_MC_STBC_DECODE_IMPL_H

#include <gnuradio/ncjt/mc_stbc_decode.h>
#include "cmatrix.h"
#include "rg_modes.h"

namespace gr::ncjt
{

class mc_stbc_decode_impl : public mc_stbc_decode
{
private:
    const int d_nrx = 2; // number of receive antennas
    const int d_ntx = 4; // 4 TX antennas for double-cluster
    int d_scnum; // number of valid subcarriers
    int d_scdata; // number data subcarriers
    int d_ndatasyms;  // total number of data OFDM symbols)
    int d_npilotsyms; // number of pilot OFDM symbols
    int d_numsyms; // total number of OFDM symbols
    int d_modtype; // modulation order
    int d_npt;  // number of tracking pilots per OFDM symbol
    int d_cpt_idx[MAX_NUM_CPT]; // cpt indices

    float d_cpe_phi1; // CPE estimation slope
    float d_cpe_phi2; // CPE estimation slope
    float d_cpe_offset1; // CPE estimation offset
    float d_cpe_offset2; // CPE estimation offset

    gr_complex *d_chan_est; // channel estimation using H-LTFs

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    add_packet_tag(uint64_t offset, int packet_len);

public:
    mc_stbc_decode_impl(int rgmode, int ndatasyms, int npilotsyms, int modtype, bool debug);
    ~mc_stbc_decode_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MC_STBC_DECODE_IMPL_H */
