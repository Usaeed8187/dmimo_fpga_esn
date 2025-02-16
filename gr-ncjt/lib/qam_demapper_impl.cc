/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "qam_demapper_impl.h"
#include <gnuradio/io_signature.h>

namespace gr::ncjt
{

qam_demapper::sptr
qam_demapper::make(int nstrms, int modtype, bool usecsi, bool debug)
{
    return gnuradio::make_block_sptr<qam_demapper_impl>(nstrms, modtype, usecsi, debug);
}

qam_demapper_impl::qam_demapper_impl(int nstrms, int modtype, bool usecsi, bool debug)
    : gr::tagged_stream_block("qam_demapper",
                              gr::io_signature::make(nstrms, 2 * nstrms, sizeof(gr_complex)),
                              gr::io_signature::make(nstrms, nstrms, sizeof(char)), "packet_len"),
      d_usecsi(usecsi), d_debug(debug)
{
    if (nstrms < 1 || nstrms > 8)
        throw std::runtime_error("only sport 1 to 8 data channels");
    d_num_strms = nstrms;
    if (modtype != 2 && modtype != 4 && modtype != 6 && modtype != 8)
    {
        throw std::runtime_error("unsupported modulation type");
    }
    d_modtype = modtype;

    set_tag_propagation_policy(block::TPP_DONT);
}

qam_demapper_impl::~qam_demapper_impl()
{
}

int
qam_demapper_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    int num_input_ports = d_usecsi ? 2 * d_num_strms : d_num_strms;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    return d_modtype * min_input_items;
}

int
qam_demapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    int num_input_ports = d_usecsi ? 2 * d_num_strms : d_num_strms;
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < num_input_ports; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    int num_inputs = std::min(noutput_items / d_modtype, min_input_items);
    if (num_inputs <= 0)
        return 0;

    for (int ch = 0; ch < d_num_strms; ch++)
    {
        const gr_complex *in = (const gr_complex *) input_items[ch];
        const gr_complex *csi = d_usecsi ? (const gr_complex *) input_items[d_num_strms + ch] : nullptr;
        char *out = (char *) output_items[ch];

        for (int i = 0; i < num_inputs; i++)
        {
            float x = in[i].real();
            float y = in[i].imag();
            if (d_modtype == 2) // QPSK
            {
                out[2 * i] = (y < 0) ? 1 : 0;
                out[2 * i + 1] = (x >= 0) ? 1 : 0;
            }
            else if (d_modtype == 4) // 16QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float h2 = d_usecsi ? 2.0 / sqrt(10.0) * csi[i].real() : 2.0 / sqrt(10.0);
                out[4 * i]     = (ym <= h2) ? 1 : 0;
                out[4 * i + 1] = (y < 0) ? 1 : 0;
                out[4 * i + 2] = (xm <= h2) ? 1 : 0;
                out[4 * i + 3] = (x >= 0) ? 1 : 0;
            }
            else if (d_modtype == 6) // 64QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(42.0);
                float h2 = d_usecsi ? a2 * csi[i].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[i].real() : 2.0 * a2;
                float h6 = d_usecsi ? 3.0 * a2 * csi[i].real() : 3.0 * a2;
                out[6 * i]     = (ym >= h2 && ym <= h6) ? 1 : 0;
                out[6 * i + 1] = (ym <= h4) ? 1 : 0;
                out[6 * i + 2] = (y < 0) ? 1 : 0;
                out[6 * i + 3] = (xm >= h2 && xm <= h6) ? 1 : 0;
                out[6 * i + 4] = (xm <= h4) ? 1 : 0;
                out[6 * i + 5] = (x >= 0) ? 1 : 0;
            }
            else if (d_modtype == 8) // 256QAM
            {
                float xm = abs(x);
                float ym = abs(y);
                float a2 = 2.0 / sqrt(170.0);
                float h2 = d_usecsi ? a2 * csi[i].real() : a2;
                float h4 = d_usecsi ? 2.0 * a2 * csi[i].real() : 2.0 * a2;
                float h8 = d_usecsi ? 4.0 * a2 * csi[i].real() : 4.0 * a2;
                out[8 * i]     = (abs(abs(ym - h8) - h4) <= h2) ? 1 : 0;
                out[8 * i + 1] = (abs(ym - h8) <= h4) ? 1 : 0;
                out[8 * i + 2] = (ym <= h8) ? 1 : 0;
                out[8 * i + 3] = (y <= 0) ? 1 : 0;
                out[8 * i + 4] = (abs(abs(xm - h8) - h4) <= h2) ? 1 : 0;
                out[8 * i + 5] = (abs(xm - h8) <= h4) ? 1 : 0;
                out[8 * i + 6] = (xm <= h8) ? 1 : 0;
                out[8 * i + 7] = (x >= 0) ? 1 : 0;
            }
        }
    }

    return num_inputs * d_modtype;
}

} /* namespace gr::ncjt */
