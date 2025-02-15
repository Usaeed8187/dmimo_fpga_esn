/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "rg_mapper_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"
#include "qam_constellation.h"

namespace gr::ncjt
{

rg_mapper::sptr
rg_mapper::make(int nstrm, int framelen, int ndatasyms, int npilotsyms, int modtype, bool debug)
{
    return gnuradio::make_block_sptr<rg_mapper_impl>(nstrm, framelen, ndatasyms, npilotsyms, modtype, debug);
}

rg_mapper_impl::rg_mapper_impl(int nstrm, int framelen, int ndatasyms, int npilotsyms, int modtype, bool debug)
    : gr::tagged_stream_block(
    "stream_mapper",
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    "packet_len"), d_debug(debug)
{
    if (nstrm < 1 || nstrm > 4)
        throw std::runtime_error("Invalid number of streams");
    d_nstrm = nstrm;

    if (ndatasyms <= 10 || ndatasyms > 200)
        throw std::runtime_error("Invalid number of data OFDM symbols");
    d_ndatasyms = ndatasyms;
    d_nsyms_per_stream = SD_NUM * d_ndatasyms;

    if (modtype != 2 && modtype != 4 && modtype != 6)
        throw std::runtime_error("Unsupported modulation mode");
    d_modtype = modtype;

    // Check input data frame length
    d_framelen = framelen;
    d_total_symbols_required = d_framelen / (d_nstrm * d_modtype);
    if (d_total_symbols_required > d_nsyms_per_stream)
        throw std::runtime_error("Data frame size too large");
    else if (d_total_symbols_required < d_nsyms_per_stream - 2 * SD_NUM)
        throw std::runtime_error("Data frame require too much padding");

    d_fftsize = 64;
    d_npt = 4;

    float cshift[4] = {0.0, -8.0, -4.0, -12.0};
    for (int ss = 0; ss < 4; ss++)
        for (int k = 0; k < SC_NUM / 2; k++)
        {
            int pn = k - SC_NUM / 2;
            int pp = k + 1;
            d_phaseshift[ss][k] = exp(gr_complex(0, -2.0 * M_PI * cshift[ss] / d_fftsize * pn));
            d_phaseshift[ss][k + SC_NUM / 2] = exp(gr_complex(0, -2.0 * M_PI * cshift[ss] / d_fftsize * pp));
        }

    set_tag_propagation_policy(block::TPP_DONT);
}

rg_mapper_impl::~rg_mapper_impl()
{
}

int
rg_mapper_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int num_blocks = ninput_items[0] / d_framelen;
    if (num_blocks >= 1)
        return SC_NUM * d_ndatasyms;
    else
        return 0;
}

int
rg_mapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
{
    auto in = static_cast<const uint8_t *>(input_items[0]);
    if (ninput_items[0] != d_framelen)
    {
        dout << "incorrect data input length: " << ninput_items[0] << std::endl;
        throw std::runtime_error("Data frame size too small");
    }

    // QAM symbols mapping for all streams and insert pilot tones
    //int max_didx = d_nstrm * d_modtype * d_total_symbols_required + d_nstrm;
    for (int ss = 0; ss < d_nstrm; ss++)
    {
        auto out = static_cast<gr_complex *>(output_items[ss]);
        int cursym = 0, scidx = 0, ptidx = 0;
        int offset, mi;
        update_pilots(-1);
        for (int didx = ss; didx < d_framelen; didx += (d_nstrm * d_modtype))
        {
            auto dbits = in + didx;
            offset = cursym * SC_NUM + scidx;
            switch (d_modtype)
            {
                case 2: // QPSK: 4 symbols per byte
                    mi = dbits[d_nstrm] + 2 * dbits[0]; // MSB first
                    out[offset] = CONST_QPSK[mi];
                    break;
                case 4: // 16QAM
                    mi = dbits[3 * d_nstrm] + 2 * dbits[2 * d_nstrm]
                        + 4 * dbits[d_nstrm] + 8 * dbits[0]; // MSB first
                    out[offset] = CONST_16QAM[mi];
                    break;
                case 6: // 64QAM
                    mi = dbits[5 * d_nstrm] + 2 * dbits[4 * d_nstrm] + 4 * dbits[3 * d_nstrm]
                        + 8 * dbits[2 * d_nstrm] + 16 * dbits[d_nstrm] + 32 * dbits[0]; // LSB first
                    out[offset] = CONST_64QAM[mi];
                    break;
                case 8: // 256QAM
                    mi = dbits[7 * d_nstrm] + 2 * dbits[6 * d_nstrm] + 4 * dbits[5 * d_nstrm] + 8 * dbits[4 * d_nstrm]
                        + 16 * dbits[3 * d_nstrm] + 32 * dbits[2*d_nstrm] + 64 * dbits[d_nstrm] + 128 * dbits[0]; // LSB first
                    out[offset] = CONST_256QAM[mi];
                    break;
            }
            if (++scidx == SC_NUM)
            {
                scidx = 0;
                cursym += 1;
            }
            if (scidx == 7 || scidx == 21 || scidx == 34 || scidx == 48)
            {
                if (scidx == 7)
                    update_pilots(cursym);
                scidx += 1;  // insert pilot tones
                out[offset + 1] = d_cur_pilot[ss][ptidx];
                if (++ptidx == d_npt)
                    ptidx = 0;
            }
        }

        while (cursym < d_ndatasyms) // padding
        {
            auto dbits = in + cursym + scidx; // random bits
            offset = cursym * SC_NUM + scidx;
            // use QPSK
            mi = dbits[1] + 2 * dbits[0]; // MSB first
            out[offset] = CONST_QPSK[mi];
            if (++scidx == SC_NUM)
            {
                scidx = 0;
                cursym += 1;
            }
            if (scidx == 7 || scidx == 21 || scidx == 34 || scidx == 48)
            {
                if (scidx == 7)
                    update_pilots(cursym);
                scidx += 1;  // insert pilot tones
                out[offset + 1] = d_cur_pilot[ss][ptidx];
                if (++ptidx == d_npt)
                    ptidx = 0;
            }
        }
    }

    // Apply cyclic shifts
    for (int ss = 1; ss < d_nstrm; ss++)
    {
        auto out = static_cast<gr_complex *>(output_items[ss]);
        for (int cursym = 0; cursym < d_ndatasyms; cursym++)
            for (int k = 0; k < SC_NUM; k++)
            {
                int offset = cursym * SC_NUM + k;
                out[offset] *= d_phaseshift[ss][k];
            }
    }

    return SC_NUM * d_ndatasyms;
}

void
rg_mapper_impl::update_pilots(int symidx)
{
    // initial state for first data symbol
    if (symidx < 0)
    {
        d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x71; // offset = 3 or 4
        return;
    }

    // update pilot parity for current symbol
    unsigned parity_bit = (d_pilot_lsfr >> 6) ^ ((d_pilot_lsfr >> 3) & 0x1);
    d_pilot_lsfr = ((d_pilot_lsfr << 1) & 0x7E) | parity_bit; // circular 7-bit shifter
    float pilot_parity = (parity_bit == 1) ? -1 : 1;

    // base pilot sequence for 64-FFT & 2/4-stream case
    const float basePilots4[6][8] = {{1, 1, -1, -1}, {1, -1, -1, 1}, // 2Rx
                                     {1, 1, 1, -1}, {1, 1, -1, 1},  // 4rx
                                     {1, -1, 1, 1}, {-1, 1, 1, 1}}; // 4Rx
    // base pilot sequence for 256-FFT single-stream case
    const float basePilots8[4][8] = {{1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1}};

    auto basePilot = (d_fftsize == 64) ? ((d_nstrm == 2) ? basePilots4 : basePilots4 + 2) : basePilots8;

    // generate current pilots
    for (int i = 0; i < d_npt; i++) // for all pilot positions
    {
        unsigned int idx = (symidx + i) % d_npt;
        for (int k = 0; k < d_nstrm; k++)  // for all streams
            d_cur_pilot[k][i] = pilot_parity * basePilot[k][idx];
    }
}

} /* namespace gr::ncjt */
