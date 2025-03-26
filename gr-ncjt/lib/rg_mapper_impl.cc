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
rg_mapper::make(int fftsize, int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms,
                int datamodtype, int ctrlmodtype, int numue, int ueidx, bool mucpt,
                const char *ltfdata, bool addltf, bool addcs, bool debug)
{
    return gnuradio::make_block_sptr<rg_mapper_impl>(
        fftsize, nstrm, framelen, ndatasyms, npilotsyms, nctrlsyms, datamodtype, ctrlmodtype,
        numue, ueidx, mucpt, ltfdata, addltf, addcs, debug);
}

rg_mapper_impl::rg_mapper_impl(int fftsize, int nstrm, int framelen, int ndatasyms, int npilotsyms, int nctrlsyms,
                               int datamodtype, int ctrlmodtype, int numue, int ueidx, bool mucpt,
                               const char *ltfdata, bool addltf, bool addcs, bool debug)
    : gr::tagged_stream_block(
    "rg_mapper",
    gr::io_signature::make(1, 1, sizeof(uint8_t)),
    gr::io_signature::make(nstrm, nstrm, sizeof(gr_complex)),
    "packet_len"), d_nltfsyms(0), d_add_ltf(addltf), d_add_cyclic_shift(addcs), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;

    if (d_fftsize == 64)
    {
        d_scnum = 56;
        d_sdnum = 52;
        d_npt = 4;
    }
    else
    {
        d_scnum = 242;
        d_sdnum = 234;
        d_npt = 8;
    }

    if (nstrm < 1 || nstrm > 4)
        throw std::runtime_error("Invalid number of streams");
    d_nstrm = nstrm;

    if (ndatasyms < 2 || ndatasyms > 200)
        throw std::runtime_error("Invalid number of data OFDM symbols");
    d_ndatasyms = ndatasyms;
    if (npilotsyms < 0 || npilotsyms > 4)
        throw std::runtime_error("Invalid number of pilot OFDM symbols");
    d_npilotsyms = npilotsyms;
    if (nctrlsyms < 0 || nctrlsyms > 50)
        throw std::runtime_error("Invalid number of control channel OFDM symbols");
    d_nctrlsyms = nctrlsyms;
    d_numofdmsyms = d_ndatasyms + d_npilotsyms + d_nctrlsyms;

    if (datamodtype != 2 && datamodtype != 4 && datamodtype != 6 && datamodtype != 8)
        throw std::runtime_error("Unsupported data modulation mode");
    d_data_modtype = datamodtype;

    if (ctrlmodtype != 2 && ctrlmodtype != 4 && ctrlmodtype != 6)
        throw std::runtime_error("Unsupported control channel modulation mode");
    d_ctrl_modtype = ctrlmodtype;

    d_frame_ctrl_len = d_nstrm * d_sdnum * d_nctrlsyms * d_ctrl_modtype;  // fixed length control channel
    d_frame_data_len = framelen; // automatic data channel padding
    int data_bits_available = d_nstrm * d_sdnum * d_ndatasyms * d_data_modtype;
    if (d_frame_data_len > data_bits_available)
        throw std::runtime_error("Data frame size too large");
    else if (d_frame_data_len < data_bits_available - d_nstrm * d_sdnum * d_data_modtype)
        throw std::runtime_error("Data frame require too much padding");

    if (numue < 1 || numue > 8)
        throw std::runtime_error("Invalid total number of UEs");
    d_numue = numue;
    if (ueidx < -1 || ueidx >= numue)
        throw std::runtime_error("Invalid UE index");
    d_ueidx = ueidx;
    d_mucpt = (d_numue > 1 && mucpt);

    // pre-generate all CPT pilots
    d_cpt_pilot = malloc_float(d_npt * d_nstrm * d_numofdmsyms);
    generate_cpt_pilots();

    // pre-generate cyclic phase shifts
    float cshift[4] = {0.0, -8.0, -4.0, -12.0};
    for (int ss = 0; ss < 4; ss++)
        for (int k = 0; k < d_scnum / 2; k++)
        {
            int pn = k - d_scnum / 2;
            int pp = k + 1;
            d_phaseshift[ss][k] = exp(gr_complex(0, -2.0 * M_PI * cshift[ss] / d_fftsize * pn));
            d_phaseshift[ss][k + d_scnum / 2] = exp(gr_complex(0, -2.0 * M_PI * cshift[ss] / d_fftsize * pp));
        }

    // load LTF data for channel estimation
    d_ltfdata_len = 0;
    if (d_add_ltf)
    {
        if ((d_ltfdata_len = read_ltf_data(ltfdata)) <= 0)
            throw std::runtime_error("failed to read LTF data");
        d_ltfdata_len /= d_nstrm;
        if (d_ltfdata_len % d_scnum != 0)
            throw std::runtime_error("LTF data length not correct");
        d_nltfsyms = d_ltfdata_len / d_scnum;
    }

    set_tag_propagation_policy(block::TPP_DONT);
}

rg_mapper_impl::~rg_mapper_impl()
{
    if (d_cpt_pilot != nullptr)
        volk_free(d_cpt_pilot);
    if (d_ltf_data)
        volk_free(d_ltf_data);
}

int
rg_mapper_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int noutput_items = d_scnum * d_numofdmsyms + d_ltfdata_len;

    return noutput_items;
}

int
rg_mapper_impl::work(int noutput_items, gr_vector_int &ninput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
{
    auto in = static_cast<const uint8_t *>(input_items[0]);

    if (ninput_items[0] != d_frame_data_len + d_frame_ctrl_len)
    {
        std::cout << "incorrect input length: " << ninput_items[0] << std::endl;
        throw std::runtime_error("input data/ctrl frame size not correct");
    }
    int num_output_items = d_scnum * d_numofdmsyms; // output length without LTF data

    int output_offset = 0;
    if (d_add_ltf)
    {
        for (int s = 0; s < d_nstrm; s++)
        {
            auto out = (gr_complex *) output_items[s];
            memcpy((void *) &out[0], &d_ltf_data[s * d_ltfdata_len], sizeof(gr_complex) * d_ltfdata_len);

        }
        output_offset += d_ltfdata_len;
        num_output_items += d_ltfdata_len;
    }

    // QAM symbols mapping for all streams and insert pilot tones
    for (int ss = 0; ss < d_nstrm; ss++)
    {
        auto out = static_cast<gr_complex *>(output_items[ss]);
        int cursym = 0, scidx = 0, ptidx = 0;
        int cur_modtype = (d_nctrlsyms > 0) ? d_ctrl_modtype : d_data_modtype;
        int offset, mi;

        // map control and data symbols
        for (int didx = ss; didx < d_frame_data_len + d_frame_ctrl_len; didx += (d_nstrm * cur_modtype))
        {
            if (cursym == d_nctrlsyms)
                cur_modtype = d_data_modtype;
            offset = output_offset + cursym * d_scnum + scidx;
            auto dbits = in + didx;
            switch (cur_modtype)
            {
                case 2: // QPSK: 4 symbols per byte
                    mi = (dbits[d_nstrm] << 1) + dbits[0]; // LSB first
                    out[offset] = CONST_QPSK[mi];
                    break;
                case 4: // 16QAM
                    mi = dbits[0] + (dbits[d_nstrm] << 1) + (dbits[2 * d_nstrm] << 2)
                        + (dbits[3 * d_nstrm] << 3); // LSB first
                    out[offset] = CONST_16QAM[mi];
                    break;
                case 6: // 64QAM
                    mi = dbits[0] + (dbits[d_nstrm] << 1) + (dbits[2 * d_nstrm] << 2)
                        + (dbits[3 * d_nstrm] << 3) + (dbits[4 * d_nstrm] << 4) + (dbits[5 * d_nstrm] << 5);
                    out[offset] = CONST_64QAM[mi];
                    break;
                case 8: // 256QAM
                    mi = dbits[0] + (dbits[d_nstrm] << 1) + (dbits[2 * d_nstrm] << 2)
                        + (dbits[3 * d_nstrm] << 3) + (dbits[4 * d_nstrm] << 4) + (dbits[5 * d_nstrm] << 5)
                        + (dbits[6 * d_nstrm] << 6) + (dbits[7 * d_nstrm] << 7);
                    out[offset] = CONST_256QAM[mi];
                    break;
            }
            if (++scidx == d_scnum)
            {
                scidx = 0;
                cursym += 1;
            }
            if (scidx == 7 || scidx == 21 || scidx == 34 || scidx == 48)
            {
                scidx += 1;  // insert pilot tones
                int pilot_pos = d_nstrm * d_npt * cursym + ss * d_npt + ptidx;
                out[offset + 1] = d_cpt_pilot[pilot_pos];
                if (++ptidx == d_npt)
                    ptidx = 0;
            }
        }

        // data padding
        while (cursym < d_ndatasyms)
        {
            auto dbits = in + cursym + 2 * scidx; // random bits
            offset = output_offset + cursym * d_scnum + scidx;
            // use QPSK
            mi = dbits[1] + 2 * dbits[0]; // MSB first
            out[offset] = CONST_QPSK[mi];
            if (++scidx == d_scnum)
            {
                scidx = 0;
                cursym += 1;
            }
            if (scidx == 7 || scidx == 21 || scidx == 34 || scidx == 48)
            {
                scidx += 1;  // insert pilot tones
                int pilot_pos = d_nstrm * d_npt * cursym + ss * d_npt + ptidx;
                out[offset + 1] = d_cpt_pilot[pilot_pos];
                if (++ptidx == d_npt)
                    ptidx = 0;
            }
        }
    }

    // Apply cyclic shifts for all OFDM symbols
    if (d_add_cyclic_shift)
        for (int ss = 1; ss < d_nstrm; ss++)
        {
            auto out = static_cast<gr_complex *>(output_items[ss]);
            for (int cursym = 0; cursym < d_nltfsyms + d_numofdmsyms; cursym++)
                for (int k = 0; k < d_scnum; k++)
                {
                    int offset = cursym * d_scnum + k;
                    out[offset] *= d_phaseshift[ss][k];
                }
        }

    return num_output_items;
}

void
rg_mapper_impl::generate_cpt_pilots()
{
    // initial state for first data symbol
    d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x71; // offset = 3 or 4

    // base pilot sequence for 64-FFT & 2/4-stream case
    const float basePilots4[6][8] = {{1, 1, -1, -1}, {1, -1, -1, 1}, // 2Rx
                                     {1, 1, 1, -1}, {1, 1, -1, 1},  // 4rx
                                     {1, -1, 1, 1}, {-1, 1, 1, 1}}; // 4Rx
    // base pilot sequence for 256-FFT single-stream case
    const float basePilots8[4][8] = {{1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1},
                                     {1, 1, 1, -1, -1, 1, 1, 1}};

    auto basePilot = (d_fftsize == 64) ? basePilots4 : basePilots8;

    for (int symidx = 0; symidx < d_numofdmsyms; symidx++)
    {
        // update pilot parity for current symbol
        unsigned parity_bit = (d_pilot_lsfr >> 6) ^ ((d_pilot_lsfr >> 3) & 0x1);
        d_pilot_lsfr = ((d_pilot_lsfr << 1) & 0x7E) | parity_bit; // circular 7-bit shifter
        float pilot_parity = (parity_bit == 1) ? -1 : 1;

        // generate current pilots
        for (int i = 0; i < d_npt; i++) // for all pilot positions
        {
            unsigned int idx = (symidx + i) % d_npt;
            for (int k = 0; k < d_nstrm; k++)  // for all streams
            {
                int sidx = (d_mucpt && d_ueidx >= 0) ? d_ueidx * d_nstrm + k : k; // TODO
                int offset = d_nstrm * d_npt * symidx + k * d_npt + i;
                // Use orthogonal pilots for MU case
                if (d_mucpt && d_ueidx >= 0 && (symidx % d_numue) != d_ueidx)
                    d_cpt_pilot[offset] = 0;
                else
                    d_cpt_pilot[offset] = pilot_parity * basePilot[sidx][idx];
            }
        }
    }
}

uint64_t
rg_mapper_impl::read_ltf_data(const char *filename)
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
    d_ltf_data = (gr_complex *) malloc(file_size);
    if (data_len != fread(d_ltf_data, sizeof(gr_complex), data_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        free(d_ltf_data);
        return 0;
    }

    return data_len;
}

} /* namespace gr::ncjt */
