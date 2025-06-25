/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "ul_precoding_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <Eigen/Dense>
#include "utils.h"

namespace gr::ncjt
{

ul_precoding::sptr
ul_precoding::make(int fftsize, int nss, int ntx, int ntx_gnb, int numltfsyms, int numdatasyms,
                   bool eigenmode, bool wideband, bool loadcsi, const char *csifile, bool debug)
{
    return gnuradio::make_block_sptr<ul_precoding_impl>(fftsize, nss, ntx, ntx_gnb, numltfsyms, numdatasyms,
                                                        eigenmode, wideband, loadcsi, csifile, debug);
}

ul_precoding_impl::ul_precoding_impl(int fftsize, int nss, int ntx, int ntx_gnb, int numltfsyms, int numdatasyms,
                                     bool eigenmode, bool wideband, bool loadcsi, const char *csifile, bool debug)
    : gr::tagged_stream_block(
    "ul_precoding",
    gr::io_signature::make(nss, nss, sizeof(gr_complex)),
    gr::io_signature::make(ntx, ntx, sizeof(gr_complex)), "packet_len"),
      d_wideband(wideband), d_loadcsi(loadcsi), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;
    d_scnum = (fftsize == 64) ? 56 : 242;

    if (nss < 1 || nss > 8)
        throw std::runtime_error("only support 1 to 8 streams");
    d_nss = nss;
    if (ntx != 2 && ntx != 4 && ntx != 6 && ntx != 8)
        throw std::runtime_error("only support 2/4/6/8 uplink transmitter antennas");
    d_ntx = ntx;
    d_nrx_ue = ntx; // number DL Rx antennas equal to UL Tx antennas
    if (ntx_gnb != 2 && ntx_gnb != 4 && ntx_gnb != 6 && ntx_gnb != 8)
        throw std::runtime_error("only support 2/4/6/8 downlink transmitter antennas");
    d_ntx_gnb = ntx_gnb;
    if (nss > ntx)
        throw std::runtime_error("number of streams must not larger than number of TX antennas");

    d_num_symbols = numltfsyms + numdatasyms;
    d_eigenmode_precoding = eigenmode;

    // allocate d_map_matrix
    size_t map_size = d_ntx * d_nss * d_scnum * sizeof(gr_complex);
    d_map_matrix = (gr_complex *) volk_malloc(map_size, volk_get_alignment());
    if (!d_map_matrix)
        throw std::runtime_error("Failed to allocate d_map_matrix");

    // Default mapping matrix using CSD (row-major layout)
    int cshift[] = {0, -8, -4, -12, -7, -13}; // cyclic shift
    for (int k = 0; k < d_scnum; k++)
        for (int m = 0; m < d_ntx; m++)
            for (int n = 0; n < d_nss; n++)
            {
                int ci = (k < d_scnum/2) ? (k - d_scnum/2) : (k - d_scnum/2 + 1);
                int si = m / d_nss;
                gr_complex cs = exp(gr_complex (0 ,-2.0*M_PI*cshift[si]*ci/double(d_fftsize)));
                d_map_matrix[k * d_ntx * d_nss + d_nss * m + n] = (m % d_nss == n) ? cs : 0.0;
            }

    // Initial loading of offline CSI data
    // Note: CSI data should use row-major layout
    if (d_eigenmode_precoding && d_loadcsi)
    {
        int dlen = d_scnum * d_ntx_gnb * d_nrx_ue * sizeof(gr_complex);
        if (read_offline_csi(csifile, dlen) > 0)
        {
            update_eigenmode_precoding(d_csi_data);
        }
    }

    message_port_register_in(pmt::mp("csi"));
    set_msg_handler(pmt::mp("csi"), [this](const pmt::pmt_t &msg) { process_csi_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

ul_precoding_impl::~ul_precoding_impl()
{
    if (d_map_matrix != nullptr)
        volk_free(d_map_matrix);
}

int
ul_precoding_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = d_num_symbols * d_scnum;
    return noutput_items;
}

void
ul_precoding_impl::process_csi_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t csi_meta = pmt::car(msg);
    pmt::pmt_t csi_blob = pmt::cdr(msg);

    if (pmt::dict_has_key(csi_meta, pmt::mp("reset")))
    {
        // Default mapping matrix (row-major layout)
        for (int k = 0; k < d_scnum; k++)
            for (int m = 0; m < d_ntx; m++)
                for (int n = 0; n < d_nss; n++)
                    d_map_matrix[k * d_ntx * d_nss + d_nss * m + n] = (n == m) ? 1.0 : 0.0;
        return;
    }
    else if (!pmt::dict_has_key(csi_meta, pmt::mp("csi")))
        throw std::runtime_error("invalid CSI data format");

    int dlen = (int) pmt::blob_length(csi_blob) / sizeof(gr_complex);
    if (dlen != d_ntx_gnb * d_nrx_ue * d_scnum)
    {
        dout << "CSI data length: " << dlen << std::endl;
        throw std::runtime_error("invalid CSI data length");
    }

    auto csidata = (const gr_complex *) pmt::blob_data(csi_blob);
    if (d_eigenmode_precoding)
        update_eigenmode_precoding(csidata);
    else
        update_svd_precoding(csidata);
}

void
ul_precoding_impl::update_svd_precoding(const gr_complex *csidata)
{
    gr::thread::scoped_lock lock(fp_mutex);

    for (int k = 0; k < d_scnum; k++)  // for all subcarriers
    {
        // Map CSI data (row major layout)
        Eigen::Map<const CMatrixX> H(&csidata[d_ntx * d_nss * k], d_ntx, d_nss);  // (Ntx x Nss)

        // Generate SVD precoding matrix
        Eigen::JacobiSVD<CMatrixX> svd;
        svd.compute(H.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        CMatrixX V(d_ntx, d_ntx);
        V = svd.matrixV();

        Eigen::Map<CMatrixX> W(&d_map_matrix[d_ntx * d_nss * k], d_ntx, d_nss);
        W = V(Eigen::all, Eigen::seq(0, d_nss - 1));
    }
}

void
ul_precoding_impl::update_eigenmode_precoding(const gr_complex *csidata)
{
    gr::thread::scoped_lock lock(fp_mutex);

    if (d_nss > 2)
        throw std::runtime_error("only support one or two streams for eigen mode precoding");

    if (!d_wideband)
    {
        for (int k = 0; k < d_scnum; k++)  // for all subcarriers
        {
            // Map CSI data (row major layout)
            Eigen::Map<const CMatrixX> H(&csidata[d_ntx_gnb * d_nrx_ue * k], d_ntx_gnb, d_nrx_ue);  // (DL Ntx x Nrx)

            // Generate eigen-mode precoding matrix
            Eigen::JacobiSVD<CMatrixX> svd;
            svd.compute(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
            CMatrixX V(d_nrx_ue, d_nrx_ue);
            V = svd.matrixV();

            Eigen::Map<CMatrixX> W(&d_map_matrix[d_ntx * d_nss * k], d_ntx, d_nss);
            // Select dominant eigenmode (first column of V) for rank-1/2 transmission
            W = V(Eigen::all, Eigen::seq(0, d_nss - 1));
        }
        return;
    }

    CMatrixX Hm(d_ntx_gnb, d_nrx_ue);
    Hm.setZero();
    // Average across subcarriers
    for (int k = 0; k < d_scnum; k++)
    {
        Eigen::Map<const CMatrixX> H(&csidata[d_ntx_gnb * d_nrx_ue * k], d_ntx_gnb, d_nrx_ue);  // (DL Ntx x Nrx)
        Hm += H;
    }
    Hm /= double(d_scnum);

    // Generate eigen-mode precoding matrix
    Eigen::JacobiSVD<CMatrixX> svd;
    svd.compute(Hm, Eigen::ComputeFullV | Eigen::ComputeFullU);
    CMatrixX V(d_nrx_ue, d_nrx_ue);
    V = svd.matrixV();
    // Set the same precoding matrix for all subcarriers
    for (int k = 0; k < d_scnum; k++)
    {
        Eigen::Map<CMatrixX> W(&d_map_matrix[d_ntx * d_nss * k], d_ntx, d_nss);
        W = V(Eigen::all, Eigen::seq(0, d_nss - 1));
    }
}

int
ul_precoding_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock lock(fp_mutex);

    // apply precoding or spatial mapping
    if (d_nss == 2 && d_ntx == 2)
        apply_mapping_2tx(input_items, output_items);
    else if (d_nss == 4 && d_ntx == 4)
        apply_mapping_4tx(input_items, output_items);
    else
        apply_mapping_ntx(input_items, output_items);

    return d_num_symbols * d_scnum;
}

void
ul_precoding_impl::apply_mapping_2tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto in1 = (const gr_complex *) input_items[1];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    for (int n = 0; n < d_num_symbols; n++)
    {
        CMatrixX Y(d_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < d_scnum; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            X(0, 0) = in0[n * d_scnum + k];
            X(1, 0) = in1[n * d_scnum + k];
            Eigen::Map<CMatrixX> Q(&d_map_matrix[d_nss * d_ntx * k], d_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;
            out0[n * d_scnum + k] = Y(0, 0);
            out1[n * d_scnum + k] = Y(1, 0);
        }
    }
}

void
ul_precoding_impl::apply_mapping_4tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto in1 = (const gr_complex *) input_items[1];
    auto in2 = (const gr_complex *) input_items[2];
    auto in3 = (const gr_complex *) input_items[3];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];
    auto out2 = (gr_complex *) output_items[2];
    auto out3 = (gr_complex *) output_items[3];

    for (int n = 0; n < d_num_symbols; n++)
    {
        CMatrixX Y(d_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < d_scnum; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            X(0, 0) = in0[n * d_scnum + k];
            X(1, 0) = in1[n * d_scnum + k];
            X(2, 0) = in2[n * d_scnum + k];
            X(3, 0) = in3[n * d_scnum + k];
            Eigen::Map<CMatrixX> Q(&d_map_matrix[d_nss * d_ntx * k], d_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            out0[n * d_scnum + k] = Y(0, 0);
            out1[n * d_scnum + k] = Y(1, 0);
            out2[n * d_scnum + k] = Y(2, 0);
            out3[n * d_scnum + k] = Y(3, 0);
        }
    }
}

void
ul_precoding_impl::apply_mapping_ntx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    for (int n = 0; n < d_num_symbols; n++)
    {
        int mtx_size = d_nss * d_ntx;
        CMatrixX Y(d_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < d_scnum; k++)
        {
            CMatrixX X(d_nss, 1); // (Nss, 1)
            for (int s = 0; s < d_nss; s++)
            {
                auto in = (const gr_complex *) input_items[s];
                X(s, 0) = in[n * d_scnum + k];
            }
            Eigen::Map<CMatrixX> Q(&d_map_matrix[mtx_size * k], d_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            for (int m = 0; m < d_ntx; m++)
            {
                auto out = (gr_complex *) output_items[m];
                out[n * d_scnum + k] = Y(m, 0);
            }
        }
    }
}

int
ul_precoding_impl::read_offline_csi(const char *filename, int csidlen)
{
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return 0;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t data_len = file_size / sizeof(gr_complex);
    if (data_len != uint64_t(csidlen))
        throw std::runtime_error("Offline CSI data length not correct");

    GR_FSEEK(d_fp, 0, SEEK_SET);
    d_csi_data = malloc_complex(file_size);
    if (data_len != fread(d_csi_data, sizeof(gr_complex), data_len, d_fp))
    {
        free(d_csi_data);
        throw std::runtime_error("failed to read file content");
    }

    return (int) data_len;
}

} /* namespace gr::mumimo */
