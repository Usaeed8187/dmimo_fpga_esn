/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "soft_demapper_impl.h"
#include <gnuradio/io_signature.h>

namespace gr::ncjt
{

soft_demapper::sptr
soft_demapper::make(int nstrm, int framelen, int modtype,
                    bool usecsi, bool debug)
{
    return gnuradio::make_block_sptr<soft_demapper_impl>(nstrm, framelen, modtype, usecsi, debug);
}

soft_demapper_impl::soft_demapper_impl(int nstrm, int framelen, int modtype,
                                       bool usecsi, bool debug)
    : gr::tagged_stream_block(
    "soft_demapper",
    gr::io_signature::make(nstrm, 2 * nstrm, sizeof(gr_complex)),
    gr::io_signature::make(1, 1, sizeof(float)),
    "packet_len"),
      d_usecsi(usecsi), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 8)
        throw std::runtime_error("only sport 1 to 8 data channels");
    d_nstrm = nstrm;
    if (modtype != 2 && modtype != 4 && modtype != 6)
    {
        throw std::runtime_error("unsupported modulation type");
    }
    d_modtype = modtype;

    d_framelen = framelen;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    set_tag_propagation_policy(block::TPP_DONT);
}

soft_demapper_impl::~soft_demapper_impl() {}

int
soft_demapper_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int num_input_ports = d_usecsi ? 2 * d_nstrm : d_nstrm;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    int num_output_items = d_nstrm * d_modtype * min_input_items * SD_NUM / SC_NUM;

    return num_output_items;
}

int
soft_demapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
{
    auto *out = (float *) output_items[0];

    int num_input_ports = d_usecsi ? 2 * d_nstrm : d_nstrm;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    int num_inputs = std::min(noutput_items / (d_nstrm * d_modtype), min_input_items);
    int num_frames = (int) (num_inputs * d_nstrm * d_modtype) / d_framelen;
    if (num_frames <= 0)
        return 0;
    int total_input_items = num_frames * d_framelen / (d_modtype * d_nstrm);

    for (int ch = 0; ch < d_nstrm; ch++)
    {
        const gr_complex *in = (const gr_complex *) input_items[ch];
        const gr_complex *csi = d_usecsi ? (const gr_complex *) input_items[d_nstrm + ch] : nullptr;

        for (int di = 0; di < total_input_items; di++)
        {
            auto dout = out + d_nstrm * d_modtype * di + ch;
            float x = in[di].real();
            float y = in[di].imag();
            if (d_modtype == 2) // QPSK
            {
                dout[0] = 2.0 * x; // (x >= 0) ? 1 : 0;
                dout[d_nstrm] = 2.0 * y; // (y >= 0) ? 1 : 0;
            }
            else if (d_modtype == 4) // 16QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(10.0) * csi[di].real() : 2.0 / sqrt(10.0);
                dout[0] = x; // (x >= 0) ? 1 : 0;
                dout[d_nstrm] = h2 - xm; // (xm <= h2) ? 1 : 0;
                dout[2 * d_nstrm] = y; // (y >= 0) ? 1 : 0;
                dout[3 * d_nstrm] = h2 - ym; // (ym <= h2) ? 1 : 0;
            }
            else // 64QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(42.0) * csi[di].real() : 2.0 / sqrt(42.0);
                float h4 = d_usecsi ? 4.0 / sqrt(42.0) * csi[di].real() : 4.0 / sqrt(42.0);
                // float h6 = d_usecsi ? 6.0 / sqrt(42.0) * csi[di].real() : 6.0 / sqrt(42.0);
                dout[0] = x; // (x >= 0) ? 1 : 0;
                dout[d_nstrm] = h4 - xm; // (xm <= h4) ? 1 : 0;
                dout[2 * d_nstrm] = xm - h2; /// (xm >= h2 && xm <= h6) ? 1 : 0;
                dout[3 * d_nstrm] = y; // (y >= 0) ? 1 : 0;
                dout[4 * d_nstrm] = h4 - ym; // (ym <= h4) ? 1 : 0;
                dout[5 * d_nstrm] = h2 - ym; /// (ym >= h2 && ym <= h6) ? 1 : 0;
            }
        }
    }

    int num_output_items = d_nstrm * d_modtype * total_input_items;
    uint64_t offset = nitems_written(0);
    add_packet_tag(offset, num_output_items);

    // add data_len tag for image sink (in bits)
    int data_len = (int) num_output_items / 100 * 42;
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("data_len"),
                 pmt::from_long(data_len),
                 _id);

    return num_output_items;
}

void
soft_demapper_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    add_item_tag(0, offset,
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(packet_len),
                 pmt::string_to_symbol(name()));
}

} /* namespace gr::ncjt */
