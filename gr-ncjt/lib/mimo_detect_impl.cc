/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "mimo_detect_impl.h"
#include <gnuradio/io_signature.h>
#include "cmatrix.h"
#include "utils.h"

namespace gr::ncjt
{

mimo_detect::sptr
mimo_detect::make(int fftsize, int nrx, int nss, int ndatasymbols, int logfreq, bool debug)
{
    return gnuradio::make_block_sptr<mimo_detect_impl>(fftsize, nrx, nss, ndatasymbols, logfreq, debug);
}

mimo_detect_impl::mimo_detect_impl(int fftsize, int nrx, int nss, int ndatasymbols, int logfreq, bool debug)
    : gr::tagged_stream_block("mimo_detect",
                              gr::io_signature::make(nrx, nrx, sizeof(gr_complex)),
                              gr::io_signature::make(nss, nss, sizeof(gr_complex)),
                              "packet_len"),
      d_num_symbols(ndatasymbols), d_logfreq(logfreq), d_debug(debug)
{
    if (fftsize == 64)
    {
        d_scnum = 56;
        d_scdata = 52;
    }
    else if (fftsize == 256)
    {
        d_scnum = 242;
        d_scdata = 234;
    }
    else
        throw std::runtime_error("Unsupported OFDM FFT size");

    if (nrx < 1 || nrx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Rx antennas");
    if (nss < 1 || nss > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 streams");
    if (nss > nrx)
        throw std::runtime_error("invalid number of streams");
    d_nrx = nrx, d_nss = nss;

    d_chan_est_buf = malloc_complex(d_nrx * d_nss * d_scnum);
    d_mmse_coef = malloc_complex(d_nss * d_nrx * d_scnum);

    d_cur_symbol = 0;
    d_total_pkts = 0;

    set_tag_propagation_policy(block::TPP_DONT);
}

mimo_detect_impl::~mimo_detect_impl()
{
    if (d_chan_est_buf != nullptr)
        volk_free(d_chan_est_buf);
    if (d_mmse_coef != nullptr)
        volk_free(d_mmse_coef);
}

int
mimo_detect_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    return d_num_symbols * d_scdata;
}

int
mimo_detect_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nrx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int ninput_syms = min_input_items / d_scnum;
    if (ninput_syms != d_nss + d_num_symbols)
        throw std::runtime_error("total number of OFDM symbols to MIMO detection not correct");

    std::vector<gr::tag_t> d_tags;
    get_tags_in_window(d_tags, 0, 0, d_scnum,
                       pmt::string_to_symbol("packet_start"));
    if (!d_tags.empty())
    {
        // save channel estimation
        for (int k = 0; k < d_scnum; k++)
            for (int n = 0; n < d_nrx; n++) // for all rx
            {
                const gr_complex *in = (const gr_complex *) input_items[n];
                for (int m = 0; m < d_nss; m++) // for all streams
                {
                    // matrix shape (num_sc, num_rx, num_ss)
                    // row major memory layout for Eigen
                    int offset = k * d_nss * d_nrx + n * d_nss + m;
                    d_chan_est_buf[offset] = in[k * d_nss + m];
                }
            }
        d_cur_symbol = 0;
        d_total_pkts += 1;
    }
    else
        throw std::runtime_error("Channel estimation not received before demapping!");

    // Get noise estimation tag
    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("noise_var"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find noise_var tag");
    float nvar = pmt::to_float(d_tags[0].value);

    // Get CPE estimation tags
    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_phi1"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_phi1 tag");
    d_cpe_phi1 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_phi2"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_phi2 tag");
    d_cpe_phi2 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_offset1"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_offset1 tag");
    d_cpe_offset1 = pmt::to_float(d_tags[0].value);

    get_tags_in_window(d_tags, 0, (ninput_syms - 1) * d_scnum, ninput_syms * d_scnum,
                       pmt::string_to_symbol("cpe_offset2"));
    if (d_tags.empty())
        throw std::runtime_error("ERROR: cannot find cpe_offset2 tag");
    d_cpe_offset2 = pmt::to_float(d_tags[0].value);

    // equalization and demapping
    int cur_sym = 0, out_sym = 0;
    for (; cur_sym < ninput_syms - d_nss; cur_sym++, out_sym++)
    {
        // update MMSE symbol detection coefficients
        if (d_nss == 2 && d_nrx == 2)
            update_mmse_coef_2rx(nvar);
        else if (d_nss == 4 && d_nrx == 4)
            update_mmse_coef_4rx(nvar);
        else
            update_mmse_coef_nrx(nvar);

        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;

            if ((d_scnum == 242)
                & (i == 6 || i == 32 || i == 74 || i == 100 || i == 141 || i == 167 || i == 209 || i == 235))
                continue;

            unsigned raddr = (cur_sym + d_nss) * d_scnum + i;
            unsigned oaddr = out_sym * d_scdata + sc_cnt;
            unsigned caddr = d_nss * d_nrx * i;
            CMatrixX X(d_nrx, 1), Y(d_nss, 1);
            Eigen::Map<CMatrixX> G(&d_mmse_coef[caddr], d_nss, d_nrx);
            for (int m = 0; m < d_nrx; m++)
            {
                const gr_complex *in = (const gr_complex *) input_items[m];
                X(m, 0) = in[raddr];
            }
            Y = G * X;
            for (int n = 0; n < d_nss; n++)
            {
                gr_complex *out = (gr_complex *) output_items[n];
                out[oaddr] = Y(n, 0);
            }
            sc_cnt += 1;
        }
        d_cur_symbol += 1;
    }

    uint64_t offset = nitems_written(0);
    add_packet_tag(offset, d_scdata * out_sym);

    return (d_scdata * out_sym);
}

void
mimo_detect_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int ch = 0; ch < d_nss; ch++)
        add_item_tag(ch, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len),
                     pmt::string_to_symbol(name()));
}

void
mimo_detect_impl::update_mmse_coef_2rx(float nvar)
{
    CMatrix2 Hc, HH;
    CMatrix2 sigma = 0.5 * nvar * CMatrix2::Identity(2, 2);

    // CPE compensation
    float cpe_phi1 = -(d_cpe_offset1 + d_cur_symbol * d_cpe_phi1);
    float cpe_phi2 = -(d_cpe_offset2 + d_cur_symbol * d_cpe_phi2);
    gr_complex cpe_comp1 = std::exp(gr_complex(0, cpe_phi1));
    gr_complex cpe_comp2 = std::exp(gr_complex(0, cpe_phi2));
    CMatrix2 C;
    C << cpe_comp1, 0, 0, cpe_comp2; // diagonal matrix

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nSTS x nRx)
        Eigen::Map<CMatrix2> H0(&d_chan_est_buf[4 * k], 2, 2);
        Eigen::Map<CMatrix2> G(&d_mmse_coef[4 * k], 2, 2);
        // CPE compensation
        CMatrix2 H = H0;
        H = H * C; // H *= cpe_comp;
        // compute HH'+N
        Hc = H.conjugate();
        HH = H * Hc + sigma;
        // G = H' * inv(HH'+N)
        G = Hc * HH.inverse();
    }
}

void
mimo_detect_impl::update_mmse_coef_4rx(float nvar)
{
    CMatrix4 Hc, HH;
    CMatrix4 sigma = 0.5 * nvar * CMatrix4::Identity(4, 4);

    // CPE compensation
    float cpe_phi = -(d_cpe_offset1 + d_cur_symbol * d_cpe_phi1);
    gr_complex cpe_comp = std::exp(gr_complex(0, cpe_phi));

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nSTS x nRx)
        Eigen::Map<CMatrix4> H0(&d_chan_est_buf[16 * k], 4, 4);
        Eigen::Map<CMatrix4> G(&d_mmse_coef[16 * k], 4, 4);
        // CPE compensation
        CMatrix4 H = H0;
        H *= cpe_comp;
        // compute HH'+N
        Hc = H.conjugate();
        HH = H * Hc + sigma;
        // G = H' * inv(HH'+N)
        G = Hc * HH.inverse();
    }
}

void
mimo_detect_impl::update_mmse_coef_nrx(float nvar)
{
    CMatrixX Ha(d_nss, d_nrx), HH(d_nss, d_nss);
    CMatrixX sigma = nvar * CMatrixX::Identity(d_nss, d_nss);

    // CPE compensation
    float cpe_phi1 = -(d_cpe_offset1 + d_cur_symbol * d_cpe_phi1);
    float cpe_phi2 = -(d_cpe_offset2 + d_cur_symbol * d_cpe_phi2);
    gr_complex cpe_comp1 = std::exp(gr_complex(0, cpe_phi1));
    gr_complex cpe_comp2 = std::exp(gr_complex(0, cpe_phi2));

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nRx x nSTS)
        Eigen::Map<CMatrixX> H0(&d_chan_est_buf[d_nrx * d_nss * k], d_nrx, d_nss);
        Eigen::Map<CMatrixX> G(&d_mmse_coef[d_nss * d_nrx * k], d_nss, d_nrx);
        // CPE compensation (for 2 streams only)
        CMatrixX H(d_nrx, d_nss);
        H(Eigen::all, 0) = H0(Eigen::all, 0) * cpe_comp1;
        H(Eigen::all, 1) = H0(Eigen::all, 1) * cpe_comp2;
        // compute HH'+N
        Ha = H.adjoint();  // nSTS x nRx
        HH = Ha * H + sigma; // nSTS x nSTS
        // G = inv(HH'+N) * H'
        G = HH.inverse() * Ha; // nSTS x nRx
    }
}

} /* namespace gr::ncjt */
