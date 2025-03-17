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
                  int modtype, bool usecsi, bool outputsyms, bool debug)
{
    return gnuradio::make_block_sptr<rg_demapper_impl>(
        nstrm, framelen, ndatasyms, npilotsyms, modtype, usecsi, outputsyms, debug);
}

rg_demapper_impl::rg_demapper_impl(int nstrm, int framelen, int ndatasyms, int npilotsyms,
                                   int modtype, bool usecsi, bool outputsyms, bool debug)
    : gr::tagged_stream_block(
    "rg_demapper",
    gr::io_signature::make(nstrm, 2 * nstrm, sizeof(gr_complex)),
    gr::io_signature::make2(1, 2, sizeof(uint8_t), sizeof(gr_complex)),
    "packet_len"),
      d_usecsi(usecsi), d_outputsyms(outputsyms), d_debug(debug)
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

void
rg_demapper_impl::select_constellation(int modtype)
{
    switch (modtype)
    {
        case 2: // QPSK
            d_constellation = CONST_QPSK;
            break;
        case 4: // 16QAM
            d_constellation = CONST_16QAM;
            break;
        case 6: // 64QAM
            d_constellation = CONST_64QAM;
            break;
        case 8: // 256QAM
            d_constellation = CONST_256QAM;
            break;
        default:throw std::runtime_error("unsupported modtype");
    }
}

int
rg_demapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    char *strmdout = (char *) output_items[0];
    gr_complex *qamsyms = nullptr;

    bool output_constl = d_outputsyms & (output_items.size() == 2);
    if (output_constl)
    {
        select_constellation(d_modtype);
        qamsyms = (gr_complex *) output_items[1];
    }

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
                char d0 = (y < 0) ? 1 : 0;
                char d1 = (x >= 0) ? 1 : 0;
                if (output_constl)
                {
                    int mi = (d1 << 1) + d0; // LSB first
                    qamsyms[d_nstrm * di + ch] = d_constellation[mi];
                }
                sdata[0] = d0;
                sdata[d_nstrm] = d1;
            }
            else if (d_modtype == 4) // 16QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(10.0) * csi[di].real() : 2.0 / sqrt(10.0);
                char d0 = (ym <= h2) ? 1 : 0;
                char d1 = (y < 0) ? 1 : 0;
                char d2 = (xm <= h2) ? 1 : 0;
                char d3 = (x >= 0) ? 1 : 0;
                sdata[0] = d0;
                sdata[d_nstrm] = d1;
                sdata[2 * d_nstrm] = d2;
                sdata[3 * d_nstrm] = d3;
                if (output_constl)
                {
                    int mi = d0 + (d1 << 1) + (d2 << 2) + (d3 << 3); // LSB first
                    qamsyms[d_nstrm * di + ch] = CONST_16QAM[mi];
                }
            }
            else if (d_modtype == 6) // 64QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(42.0);
                float h2 = d_usecsi ? a2 * csi[di].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[di].real() : 2.0 * a2;
                float h6 = d_usecsi ? 3.0 * a2 * csi[di].real() : 3.0 * a2;
                char d0 = (ym >= h2 && ym <= h6) ? 1 : 0;
                char d1 = (ym <= h4) ? 1 : 0;
                char d2 = (y <= 0) ? 1 : 0;
                char d3 = (xm >= h2 && xm <= h6) ? 1 : 0;
                char d4 = (xm <= h4) ? 1 : 0;
                char d5 = (x >= 0) ? 1 : 0;
                sdata[0] = d0;
                sdata[d_nstrm] = d1;
                sdata[2 * d_nstrm] = d2;
                sdata[3 * d_nstrm] = d3;
                sdata[4 * d_nstrm] = d4;
                sdata[5 * d_nstrm] = d5;
                if (output_constl)
                {
                    int mi = d0 + (d1 << 1) + (d2 << 2) + (d3 << 3) + (d4 << 4) + (d5 << 5);
                    qamsyms[d_nstrm * di + ch] = CONST_64QAM[mi];
                }
            }
            else if (d_modtype == 8) // 256QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(170.0);
                float h2 = d_usecsi ? a2 * csi[di].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[di].real() : 2.0 * a2;
                float h8 = d_usecsi ? 4.0 * a2 * csi[di].real() : 4.0 * a2;
                char d0 = (abs(abs(ym - h8) - h4) <= h2) ? 1 : 0;
                char d1 = (abs(ym - h8) <= h4) ? 1 : 0;
                char d2 = (ym <= h8) ? 1 : 0;
                char d3 = (y <= 0) ? 1 : 0;
                char d4 = (abs(abs(xm - h8) - h4) <= h2) ? 1 : 0;
                char d5 = (abs(xm - h8) <= h4) ? 1 : 0;
                char d6 = (xm <= h8) ? 1 : 0;
                char d7 = (x >= 0) ? 1 : 0;
                sdata[0] = d0;
                sdata[d_nstrm] = d1;
                sdata[2 * d_nstrm] = d2;
                sdata[3 * d_nstrm] = d3;
                sdata[4 * d_nstrm] = d4;
                sdata[5 * d_nstrm] = d5;
                sdata[6 * d_nstrm] = d6;
                sdata[7 * d_nstrm] = d7;
                if (output_constl)
                {
                    int mi = d0 + (d1 << 1) + (d2 << 2) + (d3 << 3)
                        + (d4 << 4) + (d5 << 5) + (d6 << 6) + (d7 << 7);
                    qamsyms[d_nstrm * di + ch] = CONST_256QAM[mi];
                }
            }
        }
    }

    int num_output_items = d_nstrm * d_modtype * total_input_items;
    uint64_t offset = nitems_written(0);
    add_packet_tag(0, offset, num_output_items);
    produce(0, num_output_items);

    if (output_constl)
    {
        int num_output_syms = (output_constl) ? d_nstrm * total_input_items : 0;
        int offset = nitems_written(1);
        add_packet_tag(1, offset, num_output_syms);
        produce(1, num_output_syms);
    }

    // add data_len tag for image sink (in bits)
    /*int data_len = (int) num_output_items / 100 * 42;
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("data_len"),
                 pmt::from_long(data_len),
                 _id);*/

    return WORK_CALLED_PRODUCE;
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
