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
ul_precoding::make(int nss, int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms, int numprecodedsyms, bool eigenmode,  bool debug)
{
    return gnuradio::make_block_sptr<ul_precoding_impl>(nss, ul_ntx, dl_ntx, dl_nrx, numhtsyms, numdatasyms, numprecodedsyms, eigenmode, debug);
}

ul_precoding_impl::ul_precoding_impl(int nss, int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms, int numprecodedsyms, bool eigenmode, bool debug)
    : gr::tagged_stream_block(
    "ul_precoding",
    gr::io_signature::make(1, 1, nss * sizeof(gr_complex)),
    gr::io_signature::make(ul_ntx, ul_ntx, sizeof(gr_complex)), "packet_len"),
      d_debug(debug)
{
    if (nss < 1 || nss > 8)
        throw std::runtime_error("only support 1 to 8 streams");
    d_nss = nss;
    if (ul_ntx != 2 && ul_ntx != 4 && ul_ntx != 6 && ul_ntx != 8)
        throw std::runtime_error("only support 2/4/6/8 transmitter antennas");
    if (dl_ntx != 2 && dl_ntx != 4 && dl_ntx != 6 && dl_ntx != 8)
        throw std::runtime_error("only support 2/4/6/8 transmitter antennas");
    if (dl_nrx != 2 && dl_nrx != 4 && dl_nrx != 6 && dl_nrx != 8)
        throw std::runtime_error("only support 2/4/6/8 transmitter antennas");
    d_ul_ntx = ul_ntx;
    d_dl_ntx = dl_ntx;
    d_dl_nrx = dl_nrx;
    if (nss > ul_ntx)
        throw std::runtime_error("number of streams must not larger than number of antennas");

    d_num_symbols = numhtsyms + numdatasyms;
    if (numprecodedsyms > numdatasyms)
        throw std::runtime_error("number of precoded symbols larger than number of data symbols");
    d_num_precoded_syms = numhtsyms + numprecodedsyms;
    d_eigenmode_precoding = eigenmode;

    d_map_matrix = malloc_complex(sizeof(gr_complex) * d_dl_nrx * d_dl_nrx);
    read_csi_data("../data/csi_frame_1.bin");

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
    int noutput_items = d_num_symbols * SC_NUM;
    return noutput_items;
}

void
ul_precoding_impl::process_csi_message(const pmt::pmt_t &msg)
{
    pmt::pmt_t csi_meta = pmt::car(msg);
    pmt::pmt_t csi_blob = pmt::cdr(msg);

    if (pmt::dict_has_key(csi_meta, pmt::mp("reset")))
    {
        for (int k = 0; k < SC_NUM; k++)
            for (int m = 0; m < d_dl_nrx; m++)
                for (int n = 0; n < d_dl_ntx; n++)
                    d_map_matrix[k * d_dl_nrx * d_dl_ntx + d_dl_ntx * m + n] = (n == m) ? 1.0 : 0.0;
        return;
    }
    else if (!pmt::dict_has_key(csi_meta, pmt::mp("csi")))
        throw std::runtime_error("invalid CSI data format");

    int dlen = (int) pmt::blob_length(csi_blob) / sizeof(gr_complex);
    if (dlen != d_dl_ntx * d_dl_nrx * SC_NUM)
    {
        dout << "CSI data length: " << dlen << std::endl;
        throw std::runtime_error("invalid CSI data length");
    }

    auto csidata = (gr_complex *) pmt::blob_data(csi_blob);
    if (d_eigenmode_precoding)
        update_eigenmode_precoding(csidata);
    else
        update_svd_precoding(csidata);
}

void
ul_precoding_impl::update_svd_precoding(gr_complex *csidata)
{
    gr::thread::scoped_lock lock(fp_mutex);

    for (int k = 0; k < SC_NUM; k++)  // for all subcarriers
    {
        // Map CSI data
        Eigen::Map<Eigen::MatrixXcf> H(&csidata[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);  // (Ntx x Nss)

        // Generate SVD precoding matrix
        Eigen::JacobiSVD<Eigen::MatrixXcf> svd;
        svd.compute(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXcf V(d_dl_nrx, d_dl_nrx);
        V = svd.matrixV();

        Eigen::Map<Eigen::MatrixXcf> W(&d_map_matrix[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);
        W = V(Eigen::all, Eigen::seq(0, d_nss - 1));
    }
}

void
ul_precoding_impl::update_eigenmode_precoding(gr_complex *csidata)
{
    gr::thread::scoped_lock lock(fp_mutex);

    if(d_nss != 1)
        throw std::runtime_error("only support one stream eigen mode precoding");

    for (int k = 0; k < SC_NUM; k++)  // for all subcarriers
    {
        // Map CSI data
        Eigen::Map<Eigen::MatrixXcf> H(&csidata[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);  // (Ntx x Nss)

        // Generate eigen-mode precoding matrix
        Eigen::JacobiSVD<Eigen::MatrixXcf> svd;
        svd.compute(H.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXcf V(d_ul_ntx, d_ul_ntx);
        V = svd.matrixV();
        // Select dominant eigenmode (first column of V) for rank-1 transmission
        Eigen::MatrixXcf V1 = V.col(0);  // [Ntx1] for rank-1 transmission

        Eigen::Map<Eigen::MatrixXcf> W(&d_map_matrix[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);
        W = V1(Eigen::all, Eigen::seq(0, d_nss - 1));
    }
}

int
ul_precoding_impl::work(int noutput_items, gr_vector_int &ninput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock lock(fp_mutex);

//    auto in0 = (const gr_complex *) input_items[0];
//    auto out0 = (gr_complex *) output_items[0];
//    auto out1 = (gr_complex *) output_items[1];

//    int num_outputs = std::min(noutput_items, ninput_items[0]);
//    memcpy(out0, &in0[0], sizeof(gr_complex) * num_outputs);
//    memset(out1, sizeof(gr_complex), num_outputs);

    // Apply spatial mapping
    if (d_nss == 2 && d_ul_ntx == 2)
        apply_mapping_2tx(input_items, output_items);
    else if (d_nss == 4 && d_ul_ntx == 4)
        apply_mapping_4tx(input_items, output_items);
    else
        apply_mapping_ntx(input_items, output_items);

    return d_num_symbols * SC_NUM;
}

void
ul_precoding_impl::apply_mapping_2tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];

    for (int n = 0; n < d_num_symbols; n++)
    {
        int mtx_size = d_dl_ntx * d_dl_nrx;
        Eigen::Matrix2cf Y(d_ul_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            Eigen::Map<const Eigen::Matrix2cf> X(&in[n * d_nss * SC_NUM + d_nss * k], d_nss, 1); // (Nss, 1)
            Eigen::Map<Eigen::Matrix2cf> Q(&d_map_matrix[mtx_size * k], d_ul_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;
            out0[n * SC_NUM + k] = Y(0, 0);
            out1[n * SC_NUM + k] = Y(1, 0);
        }
    }
}

void
ul_precoding_impl::apply_mapping_4tx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];
    auto out1 = (gr_complex *) output_items[1];
    auto out2 = (gr_complex *) output_items[2];
    auto out3 = (gr_complex *) output_items[3];

    // Apply spatial mapping
    for (int n = 0; n < d_num_symbols; n++)
    {
        Eigen::Matrix4cf Y(4, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            Eigen::Map<const Eigen::Matrix4cf> X(&in[n * d_nss * SC_NUM + d_nss * k], 4, 1); // (Nss, 1)
            Eigen::Map<Eigen::Matrix4cf> Q(&d_map_matrix[16 * k], 4, 4); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            out0[n * SC_NUM + k] = Y(0, 0);
            out1[n * SC_NUM + k] = Y(1, 0);
            out2[n * SC_NUM + k] = Y(2, 0);
            out3[n * SC_NUM + k] = Y(3, 0);
        }
    }
}

void
ul_precoding_impl::apply_mapping_ntx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (const gr_complex *) input_items[0];

    for (int n = 0; n < d_num_precoded_syms; n++)
    {
        int mtx_size = d_dl_ntx * d_dl_nrx;
        Eigen::MatrixXcf Y(d_ul_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            Eigen::Map<const Eigen::MatrixXcf> X(&in[n * d_nss * SC_NUM + d_nss * k], d_nss, 1); // (Nss, 1)
            Eigen::Map<Eigen::MatrixXcf> Q(&d_map_matrix[mtx_size * k], d_ul_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)
            for (int ss = 0; ss < d_ul_ntx; ss++)
            {
                auto out = (gr_complex *) output_items[ss];
                out[n * SC_NUM + k] = Y(ss, 0);
            }
        }
    }

    if (d_num_precoded_syms == d_num_symbols)
        return;

    for (int n = d_num_precoded_syms; n < d_num_symbols; n++)
    {
        for (int k = 0; k < SC_NUM; k++)
        {
            for (int ss = 0; ss < d_nss; ss++)
            {
                auto out = (gr_complex *) output_items[ss];
                out[n * SC_NUM + k] = in[n * d_nss * SC_NUM + d_nss * k + ss];
            }
            for (int ss = d_nss; ss < d_ul_ntx; ss++)
            {
                auto out = (gr_complex *) output_items[ss];
                out[n * SC_NUM + k] = gr_complex(0, 0);
            }
        }
    }
}

int
ul_precoding_impl::read_csi_data(const char *filename) {
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return 0;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t data_len = file_size / sizeof(gr_complex);
    if (data_len == 0)
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    d_csi_data = (gr_complex *) malloc(file_size);
    if (data_len != fread(d_csi_data, sizeof(gr_complex), data_len, d_fp)) {
        dout << "failed to read file content" << std::endl;
        free(d_csi_data);
        return 0;
    }

    return (int) data_len;
}

} /* namespace gr::mumimo */
