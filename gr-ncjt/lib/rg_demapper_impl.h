/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RG_DEMAPPER_IMPL_H
#define INCLUDED_NCJT_RG_DEMAPPER_IMPL_H

#include <gnuradio/ncjt/rg_demapper.h>

namespace gr::ncjt
{

class rg_demapper_impl : public rg_demapper
{
private:
    int d_fftsize; // OFDM FFT size
    int d_sdnum; // number of data subcarriers
    int d_nstrm; // number of streams
    int d_ndatasyms; // total number of data OFDM symbols
    int d_npilotsyms; // number of pilot OFDM symbols
    int d_nctrlsyms; // number of control channel OFDM symbols
    int d_numofdmsyms; // total number of OFDM symbols
    int d_data_modtype;   // data modulation type (number of bits per symbol)
    int d_ctrl_modtype; // control channel modulation type
    int d_frame_data_len; // data channel length per frame (in bits)
    int d_frame_ctrl_len; // control channel length per frame (in bits)
    bool d_usecsi;   // using CSI for QAM symbol demapping

    pmt::pmt_t _id;
    bool d_debug;

protected:
    int
    calculate_output_stream_length(const gr_vector_int &ninput_items);

    void
    add_ctrl_tag(int ch, uint64_t offset, int data_len);

    void
    add_packet_tag(int ch, uint64_t offset, int packet_len);

public:
    rg_demapper_impl(int fftsize, int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms,
                     int datamodtype, int ctrlmodtype, bool usecsi, bool debug);
    ~rg_demapper_impl();

    int
    work(int noutput_items, gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RG_DEMAPPER_IMPL_H */
