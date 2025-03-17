/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RG_MAPPER_IMPL_H
#define INCLUDED_NCJT_RG_MAPPER_IMPL_H

#include <gnuradio/ncjt/rg_mapper.h>

namespace gr::ncjt
{

class rg_mapper_impl : public rg_mapper
{
private:
    const int SC_NUM = 56; // valid number of subcarriers
    const int SD_NUM = 52; // number of data subcarriers
    int d_nstrm; // number of streams
    int d_frame_data_len; // input frame data length in samples
    int d_modtype; // modulation type (2-QPSK, 4-16QAM, 6-64QAM, 8-256QAM)
    int d_nltfsyms; // total number LTF symbols
    int d_ndatasyms; // total number of data OFDM symbols
    int d_npilotsyms; // number of pilot OFDM symbols
    int d_nctrlsyms; // number of control channel OFDM symbols
    int d_numofdmsyms; // total number of OFDM symbols
    int d_nqamsyms_per_stream; // QAM symbols per stream
    int d_total_symbols_required; // total number of QAM symbols for each data frame
    int d_numue; // Total number of UEs
    int d_ueidx; // UE index for MU case (default -1 for SU case)
    bool d_mucpt;  // Use orthogonal CPT pilots for multiple Tx UEs

    int d_fftsize; // FFT size
    int d_npt; // number of pilots
    bool d_add_ltf; // Add LTF before regular resource grid
    bool d_add_cyclic_shift; // add cyclic shift to data streams
    gr_complex d_phaseshift[4][56];

    unsigned d_pilot_lsfr; // LSFR state for pilot parity sequence
    float *d_cpt_pilot; // CPT pilots for the whole OFDM frame

    int d_ltfdata_len;   // LTF length in samples
    gr_complex *d_ltf_data;  // LTF data buffer

    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    generate_cpt_pilots();

    uint64_t
    read_htf_data(const char *filename);

public:
    rg_mapper_impl(int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms, int modtype,
                   int numue, int ueidx, bool mucpt, const char *ltfdata, bool addltf, bool addcs, bool debug);
    ~rg_mapper_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RG_MAPPER_IMPL_H */
