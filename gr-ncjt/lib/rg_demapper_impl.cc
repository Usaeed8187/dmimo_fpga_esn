/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "rg_demapper_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

rg_demapper::sptr
rg_demapper::make(int fftsize, int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms,
                  int datamodtype, int ctrlmodtype, bool usecsi, bool debug)
{
    return gnuradio::make_block_sptr<rg_demapper_impl>(
        fftsize, nstrm, framelen, ndatasyms, npilotsyms, nctrlsyms, datamodtype, ctrlmodtype, usecsi, debug);
}

rg_demapper_impl::rg_demapper_impl(int fftsize, int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms,
                                   int datamodtype, int ctrlmodtype, bool usecsi, bool debug)
    : gr::tagged_stream_block(
    "rg_demapper",
    gr::io_signature::make(nstrm, 2 * nstrm, sizeof(gr_complex)),
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    "packet_len"),
      d_usecsi(usecsi), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;
    d_sdnum = (d_fftsize == 64) ? 52 : 234; // TODO add FFT sizes

    if (ndatasyms < 2 || ndatasyms > 200)
        throw std::runtime_error("Invalid number of data OFDM symbols");
    d_ndatasyms = ndatasyms;
    if (npilotsyms < 0 || npilotsyms > 4)
        throw std::runtime_error("Invalid number of pilot OFDM symbols");
    d_npilotsyms = npilotsyms;
    if (nctrlsyms < 0 || nctrlsyms > 50)
        throw std::runtime_error("Invalid number of control channel OFDM symbols");
    d_nctrlsyms = nctrlsyms;
    d_numofdmsyms = d_ndatasyms + d_npilotsyms + d_nctrlsyms;

    if (nstrm < 1 || nstrm > 8)
        throw std::runtime_error("only sport 1 to 8 data channels");
    d_nstrm = nstrm;
    if (datamodtype != 2 && datamodtype != 4 && datamodtype != 6 && datamodtype != 8)
        throw std::runtime_error("unsupported modulation type");
    d_data_modtype = datamodtype;

    if (ctrlmodtype != 2 && ctrlmodtype != 4 && ctrlmodtype != 6)
        throw std::runtime_error("Unsupported control channel modulation mode");
    d_ctrl_modtype = ctrlmodtype;

    d_frame_ctrl_len = d_nstrm * d_sdnum * d_nctrlsyms * d_ctrl_modtype;  // fixed length control channel
    d_frame_data_len = framelen; // automatic data channel padding
    int data_bits_available = d_nstrm * d_sdnum * d_ndatasyms * d_data_modtype;
    if (d_frame_data_len > data_bits_available)
        throw std::runtime_error("Data frame size too large");
    else if (d_frame_data_len < data_bits_available - d_nstrm * d_sdnum * d_data_modtype)
        throw std::runtime_error("Data frame require too much padding");

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    set_tag_propagation_policy(block::TPP_DONT);
}

rg_demapper_impl::~rg_demapper_impl()
{
}

int
rg_demapper_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    return d_frame_ctrl_len + d_frame_data_len;
}

int
rg_demapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    char *strmdout = (char *) output_items[0];

    int num_input_ports = d_usecsi ? 2 * d_nstrm : d_nstrm;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    if (min_input_items != d_sdnum * d_numofdmsyms)
    {
        std::cout << "incorrect input length: " << min_input_items << std::endl;
        throw std::runtime_error("input frame size not correct");
    }
    int total_input_items = d_frame_ctrl_len / (d_ctrl_modtype * d_nstrm)
        + d_frame_data_len / (d_data_modtype * d_nstrm);

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        const gr_complex *in = (const gr_complex *) input_items[ch];
        const gr_complex *csi = d_usecsi ? (const gr_complex *) input_items[d_nstrm + ch] : nullptr;

        int cur_modtype = (d_nctrlsyms > 0) ? d_ctrl_modtype : d_data_modtype;
        int output_offset = 0;
        for (int di = 0; di < total_input_items; di++)
        {
            auto sdata = strmdout + output_offset + ch;
            output_offset += d_nstrm * cur_modtype;
            float x = in[di].real();
            float y = in[di].imag();
            if (cur_modtype == 2) // QPSK
            {
                sdata[0] = (y < 0) ? 1 : 0;
                sdata[d_nstrm] = (x >= 0) ? 1 : 0;
            }
            else if (cur_modtype == 4) // 16QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(10.0) * csi[di].real() : 2.0 / sqrt(10.0);
                sdata[0]           = (ym <= h2) ? 1 : 0;
                sdata[d_nstrm]     = (y < 0) ? 1 : 0;
                sdata[2 * d_nstrm] = (xm <= h2) ? 1 : 0;
                sdata[3 * d_nstrm] = (x >= 0) ? 1 : 0;
            }
            else if (cur_modtype == 6) // 64QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(42.0);
                float h2 = d_usecsi ? a2 * csi[di].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[di].real() : 2.0 * a2;
                float h6 = d_usecsi ? 3.0 * a2 * csi[di].real() : 3.0 * a2;
                sdata[0]           = (ym >= h2 && ym <= h6) ? 1 : 0;
                sdata[d_nstrm]     = (ym <= h4) ? 1 : 0;
                sdata[2 * d_nstrm] = (y <= 0) ? 1 : 0;
                sdata[3 * d_nstrm] = (xm >= h2 && xm <= h6) ? 1 : 0;
                sdata[4 * d_nstrm] = (xm <= h4) ? 1 : 0;
                sdata[5 * d_nstrm] = (x >= 0) ? 1 : 0;
            }
            else if (cur_modtype == 8) // 256QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(170.0);
                float h2 = d_usecsi ? a2 * csi[di].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[di].real() : 2.0 * a2;
                float h8 = d_usecsi ? 4.0 * a2 * csi[di].real() : 4.0 * a2;
                sdata[0]           = (abs(abs(ym - h8) - h4) <= h2) ? 1 : 0;
                sdata[d_nstrm]     = (abs(ym - h8) <= h4) ? 1 : 0;
                sdata[2 * d_nstrm] = (ym <= h8) ? 1 : 0;
                sdata[3 * d_nstrm] = (y <= 0) ? 1 : 0;
                sdata[4 * d_nstrm] = (abs(abs(xm - h8) - h4) <= h2) ? 1 : 0;
                sdata[5 * d_nstrm] = (abs(xm - h8) <= h4) ? 1 : 0;
                sdata[6 * d_nstrm] = (xm <= h8) ? 1 : 0;
                sdata[7 * d_nstrm] = (x >= 0) ? 1 : 0;
            }
            if (d_frame_ctrl_len > 0 && output_offset == d_frame_ctrl_len)
                cur_modtype = d_data_modtype;
        }
    }

    add_ctrl_tag(0, 0, d_frame_ctrl_len);  // add control packet tag
    add_packet_tag(0, 0, d_frame_ctrl_len + d_frame_data_len);  // add data packet tag

    return d_frame_ctrl_len + d_frame_data_len;
}

void
rg_demapper_impl::add_ctrl_tag(int ch, uint64_t offset, int data_len)
{
    add_item_tag(ch, offset,
                 pmt::string_to_symbol("ctrl_data_len"),
                 pmt::from_long(data_len),
                 _id);
}

void
rg_demapper_impl::add_packet_tag(int ch, uint64_t offset, int packet_len)
{
    add_item_tag(ch, offset,
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(packet_len),
                 _id);
}

} /* namespace gr::ncjt */
