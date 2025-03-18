/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "ofdm_mod_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
//#include "utils.h"

namespace gr::ncjt
{

// signal normalization factor = 1/sqrt(Nt*Nsc)
// #define NORM_FACTOR_2TX 0.094491118252307    // for 2-TX
// #define NORM_FACTOR_4TX 0.066815310478106    // for 4-Tx

ofdm_mod::sptr
ofdm_mod::make(int fftsize, int cplen, int ntx, float scaling, bool debug)
{
    return gnuradio::make_block_sptr<ofdm_mod_impl>(fftsize, cplen, ntx, scaling, debug);
}

ofdm_mod_impl::ofdm_mod_impl(int fftsize, int cplen, int ntx, float scaling, bool debug)
    : gr::tagged_stream_block("ofdm_mod",
                              gr::io_signature::make2(1, 1, sizeof(gr_complex), sizeof(gr_complex)),
                              gr::io_signature::make2(1, 1, sizeof(gr_complex), sizeof(gr_complex)),
                              "packet_len"), d_ifft(fftsize, 1), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;

    if (cplen <= 0 || cplen > fftsize / 4 || (cplen != 16 && cplen != 32 && cplen != 64))
        throw std::runtime_error("Unsupported OFDM CP length");
    d_cplen = cplen;
    d_symlen = d_fftsize + d_cplen;

    if (d_fftsize == 64)
    {
        d_scnum = 56;
        d_scnum_half = 28;
    }
    else
    {
        d_scnum = 242;
        d_scnum_half = 121;
    }

    if (scaling <= 0 || scaling > 1.0)
        throw std::runtime_error("invalid number of preamble symbols for training specified");

    const double normfactor = sqrt(ntx * d_scnum);
    d_scaling = scaling / normfactor;
    memset((void *) &d_ifft.get_inbuf()[0], 0, sizeof(gr_complex) * d_fftsize);

    set_tag_propagation_policy(block::TPP_DONT);
}

ofdm_mod_impl::~ofdm_mod_impl()
{
}

int
ofdm_mod_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int nblks = ninput_items[0] / d_scnum;
    return nblks * d_symlen;
}

int
ofdm_mod_impl::work(int noutput_items, gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items)
{
    auto in0 = (const gr_complex *) input_items[0];
    auto out0 = (gr_complex *) output_items[0];

    int num_blks = ninput_items[0] / d_scnum;
    // std::cout << "Input size for OFDM modulator: " << min_input_items << " (" << num_blks << ")" << std::endl;

    int nblk = 0; // output blocks
    for (int i = 0; i < num_blks; i++)
    {
        // IFFT shift
        memcpy(&d_ifft.get_inbuf()[d_fftsize / 2 + 4], &in0[nblk * d_scnum], sizeof(gr_complex) * d_scnum_half);
        memcpy(&d_ifft.get_inbuf()[1], &in0[nblk * d_scnum + d_scnum_half], sizeof(gr_complex) * d_scnum_half);
        // compute IFFT
        d_ifft.execute();
        // output scaling
        volk_32f_s32f_multiply_32f((float *) &out0[nblk * d_symlen + d_cplen],
                                   (const float *) &d_ifft.get_outbuf()[0],
                                   d_scaling,
                                   2 * d_fftsize);
        // add CP
        memcpy(&out0[nblk * d_symlen], &out0[(nblk + 1) * d_symlen - d_cplen], sizeof(gr_complex) * d_cplen);

        nblk += 1;
    }

    add_item_tag(0, nitems_written(0),
                 pmt::string_to_symbol("packet_len"),
                 pmt::from_long(nblk * d_symlen),
                 pmt::string_to_symbol(name()));
    add_item_tag(0, nitems_written(0),
                 pmt::string_to_symbol("frame_start"),
                 pmt::from_long(nblk * d_symlen),
                 pmt::string_to_symbol(name()));

    return nblk * d_symlen;
}

} /* namespace gr::ncjt */
