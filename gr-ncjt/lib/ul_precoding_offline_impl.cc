/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ul_precoding_offline_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <Eigen/Dense>
#include "utils.h"


namespace gr::ncjt
{

ul_precoding_offline::sptr
ul_precoding_offline::make(int nss, int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms, int numprecodedsyms, bool eigenmode,  const char *filename, bool debug)
{
    return gnuradio::make_block_sptr<ul_precoding_offline_impl>(nss, ul_ntx, dl_ntx, dl_nrx, numhtsyms, numdatasyms, numprecodedsyms, eigenmode, filename, debug);
}
  

/*
 * The private constructor
 */
ul_precoding_offline_impl::ul_precoding_offline_impl(
    int nss, int ul_ntx, int dl_ntx, int dl_nrx, int numhtsyms, int numdatasyms, int numprecodedsyms,
    bool eigenmode, const char *filename, bool debug)
    : gr::tagged_stream_block(
          "ul_precoding_offline",
          gr::io_signature::make(1, 1, nss * sizeof(gr_complex)),
          gr::io_signature::make(ul_ntx, ul_ntx, sizeof(gr_complex)), "packet_len"),
          d_debug(debug)
{

//   dout << "start of constructor" << std::endl;

  if (nss < 1 || nss > 8)
      throw std::runtime_error("only support 1 to 8 streams");
  d_nss = nss;
  if (ul_ntx != 2 && ul_ntx != 4 && ul_ntx != 6 && ul_ntx != 8)
      throw std::runtime_error("only support 2/4/6/8 uplink transmitter antennas");
  if (dl_ntx != 2 && dl_ntx != 4 && dl_ntx != 6 && dl_ntx != 8)
      throw std::runtime_error("only support 2/4/6/8 downlink transmitter antennas");
  if (dl_nrx != 2 && dl_nrx != 4 && dl_nrx != 6 && dl_nrx != 8)
      throw std::runtime_error("only support 2/4/6/8 downlink reciever antennas");
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
  
  // Allocate d_map_matrix
  size_t map_size = d_dl_ntx * d_dl_nrx * SC_NUM * sizeof(gr_complex);
  d_map_matrix = (gr_complex*)volk_malloc(map_size, volk_get_alignment());
  if (!d_map_matrix) {
    throw std::runtime_error("Failed to allocate d_map_matrix");
  }
  
  d_csi_data = nullptr;
  if ((d_csi_data_len = read_csi_data(filename)) <= 0)
      throw std::runtime_error("failed to read frame data");

  set_tag_propagation_policy(block::TPP_DONT);

//   dout << "end of constructor" << std::endl;
}

/*
 * Our virtual destructor.
 */
ul_precoding_offline_impl::~ul_precoding_offline_impl() 
{
  if (d_csi_data)
        volk_free(d_csi_data);
  if (d_map_matrix != nullptr)
        volk_free(d_map_matrix);
}

int ul_precoding_offline_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items) {
  
    int noutput_items = d_num_symbols * SC_NUM;
    return noutput_items;

}

void
ul_precoding_offline_impl::update_eigenmode_precoding(gr_complex *csidata)
{
    // dout << "inside update eigenmode precoding" << std::endl;

    // gr::thread::scoped_lock lock(fp_mutex);

    // dout << "really inside update eigenmode precoding" << std::endl;

    if(d_nss != 1)
        throw std::runtime_error("only support one stream eigen mode precoding");
    
    if (!csidata) {
        dout << "csi data is null" << std::endl;
        throw std::runtime_error("CSI data pointer is null");
    }

    if (!d_map_matrix) {
        dout << "d_map_matrix is null" << std::endl;
        throw std::runtime_error("d_map_matrix is not allocated");
    }

    // Check if csidata has enough space
    size_t required_size = d_dl_ntx * d_dl_nrx * SC_NUM;
    if (d_csi_data_len < required_size) {
        dout << "csi data too small: " << d_csi_data_len << " < " << required_size << std::endl;
        throw std::runtime_error("CSI data array is too small for all subcarriers");
    }

    for (int k = 0; k < SC_NUM; k++)  // for all subcarriers
    {
        // Map CSI data
        // dout << "about to read csi data" << std::endl;
        Eigen::Map<Eigen::MatrixXcf> H(&csidata[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);  // (DL Ntx x Nrx)

        // Generate eigen-mode precoding matrix
        // dout << "generating precoding matrix" << std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXcf> svd;
        svd.compute(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXcf V(d_ul_ntx, d_ul_ntx);
        V = svd.matrixV();

        if (k==0) {
            dout << "H:" << std::endl << H << std::endl << std::endl;
            dout << "S: " << std::endl << svd.singularValues() << std::endl << std::endl;
            dout << "V:" << std::endl << V << std::endl << std::endl;
            
        }
        // Select dominant eigenmode (first column of V) for rank-1 transmission
        // dout << "selecting first column of matrix" << std::endl;
        Eigen::MatrixXcf V1 = V.col(0);  // [Ntx1] for rank-1 transmission

        // dout << "mapping precoding matrix to memory segment" << std::endl;
        Eigen::Map<Eigen::MatrixXcf> W(&d_map_matrix[d_dl_ntx * d_dl_nrx * k], d_dl_ntx, d_dl_nrx);
        // dout << "saving right singular vectors in precoding matrix memory segment" << std::endl;
        W = V1(Eigen::all, Eigen::seq(0, d_nss - 1));
    }

    // dout << "getting out of update eigenmode precoding" << std::endl;
}



int ul_precoding_offline_impl::work(int noutput_items,
                                    gr_vector_int &ninput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items) 
{
  gr::thread::scoped_lock lock(fp_mutex);
                                    
  int num_symbols = ninput_items[0] / SC_NUM;
  // dout << "num_symbols: "
  //     << (double) num_symbols << std::endl;
  // dout << "d_num_symbols: "
  //     << (double) d_num_symbols << std::endl;

  if (num_symbols != d_num_symbols)
      // dout << "num_symbols: "
      //     << (double) num_symbols << std::endl;
      // dout << "d_num_symbols: "
      //     << (double) d_num_symbols << std::endl;
      throw std::runtime_error("input data length not correct");

  // Apply spatial mapping
  if (d_nss == 2 && d_ul_ntx == 2)
    throw std::runtime_error("input data length not correct");
  else if (d_nss == 4 && d_ul_ntx == 4)
    throw std::runtime_error("input data length not correct");
  else
      apply_mapping_ntx(input_items, output_items);
//   dout << "at the end" << std::endl;

  return d_num_symbols * SC_NUM;
}

void
ul_precoding_offline_impl::apply_mapping_ntx(gr_vector_const_void_star &input_items,
                                     gr_vector_void_star &output_items)
{
    auto in = (gr_complex *) input_items[0];

    // dout << "in apply_mapping_ntx" << std::endl;

    // dout << std::endl << "about to apply precoding" << std::endl;

    for (int n = 0; n < d_num_precoded_syms; n++)
    {

        // dout << "precoding the " << n << "th symbol" << std::endl;
        int mtx_size = d_dl_ntx * d_dl_nrx;
        int section_size = mtx_size * SC_NUM;
        Eigen::MatrixXcf Y(d_ul_ntx, 1);  // (Nt, 1)
        for (int k = 0; k < SC_NUM; k++)
        {
            // dout << "precoding the " << k << "th subcarrier" << std::endl;
            // Check if data is available
            if (d_csi_data == nullptr || d_csi_data_len == 0)
            {
                // dout << "No data available to process" << std::endl;
                return;
            }

            if (d_channel_idx * section_size >= d_csi_data_len)
            {
                // dout << "completed one d_csi_data_len" << std::endl;
                d_channel_idx = 0;
            }
            
            // dout << "picked the " << d_channel_idx << "th channel" << std::endl;

            

            gr_complex *csidata = &d_csi_data[d_channel_idx * section_size];
            if (n == 0 && k ==0) 
            {
                dout << "d_channel_idx : " << d_channel_idx << std::endl;
                dout << "section_size : " << section_size << std::endl;
                dout << "d_channel_idx * section_size : " << d_channel_idx * section_size << std::endl;
                dout << "csidata(3) : " << csidata[3] << std::endl;
            }
            d_channel_idx += 1;
            // dout << "about to update precoding" << std::endl;
            update_eigenmode_precoding(csidata);
            
            // dout << "about to apply precoding" << std::endl;
            Eigen::Map<Eigen::MatrixXcf> X(&in[n * d_nss * SC_NUM + d_nss * k], d_nss, 1); // (Nss, 1)
            Eigen::Map<Eigen::MatrixXcf> Q(&d_map_matrix[mtx_size * k], d_ul_ntx, d_nss); // (Nt, Nss)
            Y = Q * X;  // (Nt,Nss) * (Nss,1)

            if (n==1 && k==0) {
                dout << "actual precoder used:" << std::endl << Q << std::endl << std::endl;
            }
    
            // dout << "about to output precoded symbols" << std::endl;
            for (int ss = 0; ss < d_ul_ntx; ss++)
            {
                auto out = (gr_complex *) output_items[ss];
                out[n * SC_NUM + k] = Y(ss, 0);
            }
        }
    }

    // dout << "precoding applied" << std::endl;

    if (d_num_precoded_syms == d_num_symbols)
        return;
    
    dout << "didn't return as expected" << std::endl;
  }

uint64_t
ul_precoding_offline_impl::read_csi_data(const char *filename)
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
    if (data_len == 0)
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    // dout << "about to allocate memory for csi data" << std::endl;
    d_csi_data = (gr_complex *)volk_malloc(file_size, volk_get_alignment());
    if (!d_csi_data) {
        dout << "failed to allocate memory for csi data" << std::endl;
        fclose(d_fp);
        return 0;
    }
    if (data_len != fread(d_csi_data, sizeof(gr_complex), data_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        volk_free(d_csi_data);
        d_csi_data = nullptr;
        fclose(d_fp);
        return 0;
    }

    fclose(d_fp);
    // dout << "finished reading csi data" << std::endl;
    dout << "csi data: " << d_csi_data[3] << std::endl;
    return data_len;
}


} /* namespace gr::ncjt */
