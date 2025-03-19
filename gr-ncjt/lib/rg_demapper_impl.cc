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
rg_demapper::make(int nstrm, int framelen, int ndatasyms, int npilotsyms,
                  int modtype, bool usecsi, bool debug)
{
    return gnuradio::make_block_sptr<rg_demapper_impl>(
        nstrm, framelen, ndatasyms, npilotsyms, modtype, usecsi, debug);
}

rg_demapper_impl::rg_demapper_impl(int nstrm, int framelen, int ndatasyms, int npilotsyms,
                                   int modtype, bool usecsi, bool debug)
    : gr::tagged_stream_block(
    "rg_demapper",
    gr::io_signature::make(nstrm, 2 * nstrm, sizeof(gr_complex)),
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    "packet_len"),
      d_usecsi(usecsi), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 8)
        throw std::runtime_error("only sport 1 to 8 data channels");
    d_nstrm = nstrm;
    if (modtype != 2 && modtype != 4 && modtype != 6 && modtype != 8)
        throw std::runtime_error("unsupported modulation type");
    d_modtype = modtype;

    d_framelen = framelen;

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
    /*int num_input_ports = d_usecsi ? 2 * d_nstrm : d_nstrm;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);*/

    int num_output_items = d_framelen;

    return num_output_items;
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

    int num_frames = (int) (min_input_items * d_nstrm * d_modtype) / d_framelen;
    if (num_frames <= 0)
    {
        dout << "input data size (" << min_input_items << ") less than one frame" << std::endl;
        return 0;
    }
    int total_input_items = num_frames * d_framelen / (d_modtype * d_nstrm);

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        const gr_complex *in = (const gr_complex *) input_items[ch];
        const gr_complex *csi = d_usecsi ? (const gr_complex *) input_items[d_nstrm + ch] : nullptr;

        for (int di = 0; di < total_input_items; di++)
        {
            auto sdata = strmdout + d_nstrm * d_modtype * di + ch;
            float x = in[di].real();
            float y = in[di].imag();
            if (d_modtype == 2) // QPSK
            {
                sdata[0] = (y < 0) ? 1 : 0;
                sdata[d_nstrm] = (x >= 0) ? 1 : 0;
            }
            else if (d_modtype == 4) // 16QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(10.0) * csi[di].real() : 2.0 / sqrt(10.0);
                sdata[0]           = (ym <= h2) ? 1 : 0;
                sdata[d_nstrm]     = (y < 0) ? 1 : 0;
                sdata[2 * d_nstrm] = (xm <= h2) ? 1 : 0;
                sdata[3 * d_nstrm] = (x >= 0) ? 1 : 0;
            }
            else if (d_modtype == 6) // 64QAM
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
            else if (d_modtype == 8) // 256QAM
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
        }
    }

    int num_output_items = d_nstrm * d_modtype * total_input_items;
    uint64_t offset = nitems_written(0);
    add_packet_tag(0, offset, num_output_items);

    // add data_len tag for image sink (in bits)
    /*int data_len = (int) num_output_items / 100 * 42;
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("data_len"),
                 pmt::from_long(data_len),
                 _id);*/

    return num_output_items;
}

void
rg_demapper_impl::add_packet_tag(int ch, uint64_t offset, int packet_len)
{
    add_item_tag(ch, offset,
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(packet_len),
                 pmt::string_to_symbol(name()));
}

} /* namespace gr::ncjt */
