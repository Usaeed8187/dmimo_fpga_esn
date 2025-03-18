/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "sic_detect_impl.h"
#include <gnuradio/io_signature.h>
#include "cmatrix.h"
#include "utils.h"
#include "qam_constellation.h"

namespace gr::ncjt
{

sic_detect::sptr
sic_detect::make(int fftsize, int nrx, int nss, int modtype, int ndatasymbols, bool debug)
{
    return gnuradio::make_block_sptr<sic_detect_impl>(
        fftsize, nrx, nss, modtype, ndatasymbols, debug);
}

sic_detect_impl::sic_detect_impl(int fftsize, int nrx, int nss, int modtype, int ndatasymbols, bool debug)
    : gr::tagged_stream_block(
    "sic_detect",
    gr::io_signature::make(nrx, nrx, sizeof(gr_complex)),
    gr::io_signature::make(nss, nss, sizeof(gr_complex)),
    "packet_len"),
      d_num_symbols(ndatasymbols), d_debug(debug)
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

    d_modtype = modtype;
    switch (modtype)
    {
        case 2:QAM_CONST = CONST_QPSK;
            break;
        case 4:QAM_CONST = CONST_16QAM;
            break;
        case 6:QAM_CONST = CONST_64QAM;
            break;
        case 8:QAM_CONST = CONST_256QAM;
            break;
        default:throw std::runtime_error("invalid modulation order");
    }

    d_chan_est_buf = malloc_complex(d_nrx * d_nss * d_scnum);
    d_mmse_weight = malloc_complex(d_nss * d_nrx * d_scnum);
    d_chan_coef = malloc_complex(d_nss * d_nrx * d_scnum);
    d_sic_order = malloc_int(d_nss * d_scnum);

    d_chan_est_ready = false;
    d_cur_symbol = 0;
    d_total_pkts = 0;

    set_tag_propagation_policy(block::TPP_DONT);
}

sic_detect_impl::~sic_detect_impl()
{
    if (d_chan_est_buf != nullptr)
        volk_free(d_chan_est_buf);
    if (d_mmse_weight != nullptr)
        volk_free(d_mmse_weight);
    if (d_chan_coef != nullptr)
        volk_free(d_chan_coef);
    if (d_sic_order != nullptr)
        free(d_sic_order);
}

int
sic_detect_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    return d_num_symbols * d_scdata;
}

int
sic_detect_impl::work(int noutput_items, gr_vector_int &ninput_items,
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
    check_cpe_tags(ninput_syms);

    // equalization and demapping
    int cur_sym = 0, out_sym = 0;
    for (; cur_sym < ninput_syms - d_nss; cur_sym++, out_sym++)
    {
        // MMSE symbol detection and CSI
        update_mmse_sic_coef(nvar);

        int sc_cnt = 0;
        for (int i = 0; i < d_scnum; i++)
        {
            if ((d_scnum == 56) & (i == 7 || i == 21 || i == 34 || i == 48))
                continue;

            if ((d_scnum == 242)
                & (i == 6 || i == 32 || i == 74 || i == 100 || i == 141 || i == 167 || i == 209 || i == 235))
                continue;

            unsigned raddr = (cur_sym + d_nss) * d_scnum + i;
            CMatrixX ry(d_nrx, 1);
            // copy received symbols
            for (int m = 0; m < d_nrx; m++)
            {
                const gr_complex *in = (const gr_complex *) input_items[m];
                ry(m, 0) = in[raddr];
            }
            // SIC detection
            int caddr = d_nss * d_nrx * i;
            Eigen::Map<const CMatrixX> G(&d_mmse_weight[caddr], d_nss, d_nrx);
            Eigen::Map<CMatrixX> C(&d_chan_coef[caddr], d_nss, d_nrx);
            for (int n = 0; n < d_nss; n++)
            {
                CMatrixX z = G.row(n) * ry;  // decision variable for best stream
                gr_complex xh = z(0, 0);
                // hard decision
                gr_complex xs = qam_detect(xh);
                ry = ry - xs * C.row(n).transpose();
                // output equalized symbols
                int sidx = d_sic_order[d_nss * i + n];
                gr_complex *out = (gr_complex *) output_items[sidx];
                unsigned oaddr = out_sym * d_scdata + sc_cnt;
                out[oaddr] = xh;
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
sic_detect_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int ch = 0; ch < d_nss; ch++)
        add_item_tag(ch, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len),
                     pmt::string_to_symbol(name()));
}

void
sic_detect_impl::check_cpe_tags(int ninput_syms)
{
    std::vector<gr::tag_t> d_tags;
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
}

void
sic_detect_impl::update_mmse_sic_coef(float nvar)
{
    // clear channel coefficients
    std::memset((void *) d_chan_coef, 0, sizeof(gr_complex) * d_nss * d_nrx * d_scnum);

    // CPE compensation
    float cpe_phi1 = -(d_cpe_offset1 + d_cur_symbol * d_cpe_phi1);
    float cpe_phi2 = -(d_cpe_offset2 + d_cur_symbol * d_cpe_phi2);
    gr_complex cpe_comp1 = std::exp(gr_complex(0, cpe_phi1));
    gr_complex cpe_comp2 = std::exp(gr_complex(0, cpe_phi2));

    for (int k = 0; k < d_scnum; k++)
    {
        // get channel estimation for current subcarrier (nRx x nSTS)
        Eigen::Map<const CMatrixX> H0(&d_chan_est_buf[d_nrx * d_nss * k], d_nrx, d_nss);
        // CPE compensation (for 2 UEs only)
        CMatrixX H(d_nrx, d_nss);
        for (int k = 0; k < d_nss / 2; k++)
        {
            H(Eigen::all, k) = H0(Eigen::all, k) * cpe_comp1;
            H(Eigen::all, k + d_nss / 2) = H0(Eigen::all, k + d_nss / 2) * cpe_comp2;
        }

        std::vector<int> stream_indices(d_nss); // original indices for remaining stream
        for (int n = 0; n < d_nss; n++)
            stream_indices[n] = n;

        int nss = d_nss;  // number of remaining streams
        for (int n = 0; n < d_nss; n++)
        {
            int min_idx;
            CMatrixX Ha(nss, d_nrx), HH(nss, nss);
            CMatrixX sigma = nvar * CMatrixX::Identity(nss, nss);
            CMatrixX G(nss, d_nrx);
            Eigen::VectorXf Gp(nss, 1);
            Eigen::Map<CMatrixX> W(&d_mmse_weight[d_nrx * d_nss * k + n * d_nrx], 1, d_nrx);
            Eigen::Map<CMatrixX> C(&d_chan_coef[d_nrx * d_nss * k + n * d_nrx], d_nrx, 1);

            // MMSE equalization
            Ha = H.adjoint();  // nSTS x nRx
            HH = Ha * H + sigma; // compute HH'+N, nSTS x nSTS
            G = HH.inverse() * Ha; // G = inv(HH'+N) * H', nSTS x nRx

            //  find the best Tx stream
            Gp = G.cwiseAbs2().rowwise().sum();
            Gp.minCoeff(&min_idx);
            W = G.row(min_idx); // equalization weights for the best stream
            int sidx = stream_indices[min_idx]; // original index for the best stream
            d_sic_order[d_nss * k + n] = sidx;
            C = H.col(min_idx); // save the MMSE coefficients

            // Update channel coefficient matrix
            nss = nss - 1;
            if (nss > 0)
            {
                // remove corresponding column from channel matrix
                if (min_idx < nss)
                    H.block(0, min_idx, d_nrx, nss - min_idx) = H.rightCols(nss - min_idx);
                H.conservativeResize(d_nrx, nss);
                stream_indices.erase(stream_indices.begin() + min_idx);
            }
        }
    }
}

gr_complex
sic_detect_impl::qam_detect(const gr_complex yh)
{
    int bits[8]; // detect bits
    int si; // QAM symbol index

    float x = yh.real();
    float y = yh.imag();
    if (d_modtype == 2) // QPSK
    {
        bits[0] = (y < 0) ? 1 : 0;
        bits[1] = (x >= 0) ? 1 : 0;
        si = bits[0] + (bits[1] << 1);
    }
    else if (d_modtype == 4) // 16QAM
    {
        float xm = abs(x);
        float ym = abs(y);
        float h2 = 2.0 / sqrt(10.0);
        bits[0] = (ym <= h2) ? 1 : 0;
        bits[1] = (y < 0) ? 1 : 0;
        bits[2] = (xm <= h2) ? 1 : 0;
        bits[3] = (x >= 0) ? 1 : 0;
        si = bits[0] + (bits[1] << 1) + (bits[2] << 2) + (bits[3] << 3);
    }
    else if (d_modtype == 6) // 64QAM
    {
        float xm = abs(x);
        float ym = abs(y);
        float h2 = 2.0 / sqrt(42.0);
        float h4 = 2.0 * h2;
        float h6 = 3.0 * h2;
        bits[0] = (ym >= h2 && ym <= h6) ? 1 : 0;
        bits[1] = (ym <= h4) ? 1 : 0;
        bits[2] = (y < 0) ? 1 : 0;
        bits[3] = (xm >= h2 && xm <= h6) ? 1 : 0;
        bits[4] = (xm <= h4) ? 1 : 0;
        bits[5] = (x >= 0) ? 1 : 0;
        si = bits[0] + (bits[1] << 1) + (bits[2] << 2)
            + (bits[3] << 3) + (bits[4] << 4) + (bits[5] << 5);
    }
    else if (d_modtype == 8) // 256QAM
    {
        float xm = abs(x);
        float ym = abs(y);
        float h2 = 2.0 / sqrt(170.0);
        float h4 = 2.0 * h2;
        float h8 = 4.0 * h2;
        bits[0] = (abs(abs(ym - h8) - h4) <= h2) ? 1 : 0;
        bits[1] = (abs(ym - h8) <= h4) ? 1 : 0;
        bits[2] = (ym <= h8) ? 1 : 0;
        bits[3] = (y <= 0) ? 1 : 0;
        bits[4] = (abs(abs(xm - h8) - h4) <= h2) ? 1 : 0;
        bits[5] = (abs(xm - h8) <= h4) ? 1 : 0;
        bits[6] = (xm <= h8) ? 1 : 0;
        bits[7] = (x >= 0) ? 1 : 0;
        si = bits[0] + (bits[1] << 1) + (bits[2] << 2) + (bits[3] << 3)
            + (bits[4] << 4) + (bits[5] << 5) + (bits[6] << 6) + (bits[7] << 7);
    }

    return QAM_CONST[si];
}

} /* namespace gr::ncjt */
