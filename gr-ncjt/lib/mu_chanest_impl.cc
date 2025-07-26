/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "mu_chanest_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "utils.h"

namespace gr::ncjt
{

mu_chanest::sptr
mu_chanest::make(int rgmode, int ntx, int nrx, int nue, int npreamblesyms, int ndatasyms,
                 bool mucpt, bool removecs, int logfreq, bool debug)
{
    return gnuradio::make_block_sptr<mu_chanest_impl>(
        rgmode, ntx, nrx, nue, npreamblesyms, ndatasyms, mucpt, removecs, logfreq, debug);
}

mu_chanest_impl::mu_chanest_impl(int rgmode, int ntx, int nrx, int nue,
                                 int npreamblesyms, int ndatasyms, bool mucpt, bool removecs, int logfreq, bool debug)
    : gr::tagged_stream_block(
    "mu_chanest",
    gr::io_signature::make(1, 8, sizeof(gr_complex)),
    gr::io_signature::make(1, 8, sizeof(gr_complex)),
    "packet_len"),
      d_preamble_symbols(npreamblesyms), d_data_symbols(ndatasyms), d_remove_cyclic_shift(removecs),
      d_total_frames(0), d_reset_frames(0), d_logfreq(logfreq), d_debug(debug)
{
    if (rgmode < 0 || rgmode >= 8)
        throw std::runtime_error("Unsupported RG mode");

    d_fftsize = RG_FFT_SIZE[rgmode];
    d_scnum = RG_NUM_VALID_SC[rgmode];
    d_ncpt = RG_NUM_CPT[rgmode];
    for (int k = 0; k < MAX_NUM_CPT; k++)
        d_cpt_idx[k] = RG_CPT_INDX[rgmode][k];

    if (rgmode < 2)
        NORM_LTF_SEQ = NORM_LTF_SEQ_64;
    else if (rgmode < 4)
        NORM_LTF_SEQ = NORM_LTF_SEQ_256A;
    else
        NORM_LTF_SEQ = NORM_LTF_SEQ_256B;

    if (ntx < 1 || ntx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Tx antennas");
    d_nrx = nrx;
    if (nrx < 1 || nrx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Rx antennas");
    d_ntx = ntx;

    if (nue < 1 || nue > ntx || nue > nrx || ntx % nue != 0)
        throw std::runtime_error("invalid number of UEs specified");
    d_nue = nue;
    d_mucpt = (d_nue > 1 && mucpt);

    d_nss = d_ntx / d_nue;  // number of streams per UE
    if (d_nss != 1 && d_nss != 2 && d_nss != 4)
        throw std::runtime_error("currently only support 1, 2 or 4 multiplexing streams");

    d_cpt_pilot = malloc_float(d_ncpt * d_ntx * d_data_symbols);
    d_cpe_est = malloc_float(d_data_symbols);
    d_est_pilots = malloc_complex(d_ncpt * d_nrx * d_data_symbols);
    d_chan_est = malloc_complex(d_ntx * d_nrx * d_scnum);
    d_cshift = malloc_complex(4 * d_scnum);

    // pre-generate all CPT pilots
    generate_cpt_pilots();

    // cyclic shift compensation
    int scidx_offset_1 = (d_fftsize == 64) ? d_scnum / 2 : (d_scnum / 2 + 1);
    int scidx_offset_2 = (d_fftsize == 64) ? (d_scnum / 2 - 1) : (d_scnum / 2 - 2);
    float cshift[4] = {0, -8, -4, -12};
    for (int m = 0; m < 4; m++)
    {
        for (int k = 0; k < d_scnum / 2; k++)
            d_cshift[m * d_scnum + k] = exp(gr_complex(0, 2.0 * M_PI / d_fftsize * cshift[m] * (k - scidx_offset_1)));
        for (int k = d_scnum / 2; k < d_scnum; k++)
            d_cshift[m * d_scnum + k] = exp(gr_complex(0, 2.0 * M_PI / d_fftsize * cshift[m] * (k - scidx_offset_2)));
    }

    // Pd: nSTS x nLTF
    if (d_nss == 2) // 2Tx2Rx, 2Tx4Rx
    {
        d_Pd.resize(2, 2);
        d_Pd << 1, -1, 1, 1; // column major
    }
    else if (d_nss == 4) // 4Tx4Rx
    {
        d_Pd.resize(4, 4);
        d_Pd << 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1; // column major
    }

    // const float normfactor = sqrt(d_scnum / float(d_ntx * d_fftsize));
    const float normfactor = 1.0f / float(d_ntx);
    d_Pd = normfactor * d_Pd;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    message_port_register_out(pmt::mp("cpe"));
    message_port_register_out(pmt::mp("csi"));

    set_tag_propagation_policy(block::TPP_DONT);
}

mu_chanest_impl::~mu_chanest_impl()
{
    if (d_cpt_pilot != nullptr)
        volk_free(d_cpt_pilot);
    if (d_cpe_est != nullptr)
        volk_free(d_cpe_est);
    if (d_est_pilots != nullptr)
        volk_free(d_est_pilots);
    if (d_chan_est != nullptr)
        volk_free(d_chan_est);
    if (d_cshift != nullptr)
        volk_free(d_cshift);
}

int
mu_chanest_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    return (d_ntx + d_data_symbols) * d_scnum;
}

int
mu_chanest_impl::work(int noutput_items, gr_vector_int &ninput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nrx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int ninput_syms = min_input_items / d_scnum;
    if (ninput_syms != d_preamble_symbols + d_data_symbols)
        throw std::runtime_error("total number of OFDM symbols to channel estimation not correct");

    int noutput_syms = d_ntx + d_data_symbols;
    if (d_nss == 1) // d_nue >= 1
        ltf_chan_est_1tx(input_items, output_items, 0);
    else if (d_nss == 2 && d_nue == 2)
        ltf_chan_est_2tx(input_items, output_items, 0);
    else if (d_nss == 2 && d_nue == 1)
        ltf_chan_est_2rx(input_items, output_items, 0);
    else
        ltf_chan_est_nrx(input_items, output_items, 0);

    uint64_t offset = nitems_written(0);
    add_frame_tag(offset);

    // CPE estimation and compensation
    cpe_estimate_comp(input_items, output_items, d_nss);

    d_total_frames += 1;
    if (d_reset_frames > 0)
        d_reset_frames -= 1;

    d_sigpwr_est = (float) d_sigpwr_sum / (float) d_data_symbols;
    d_noise_est = (float) d_noise_sum / (float) d_data_symbols;
    auto toffset = nitems_written(0) + (noutput_syms - 1) * d_scnum;
    add_power_noise_tag(toffset, d_sigpwr_est, d_noise_est);
    add_cpe_est_tag(toffset, d_cpe_phi1, d_cpe_offset1, d_cpe_phi2, d_cpe_offset2);

    auto snr_db = 10.0 * std::log10(d_sigpwr_est / d_noise_est);
    if (d_total_frames % d_logfreq == 0)
        dout << ">>>>>>>> Signal power: " << d_sigpwr_est << "  noise: " << d_noise_est
             << " (SNR " << snr_db << " dB)" << std::endl;

    // Copy received symbols to output
    for (int m = 0; m < d_nrx; m++)
    {
        auto *in = (const gr_complex *) input_items[m] + d_preamble_symbols * d_scnum;
        auto *out = (gr_complex *) output_items[m] + d_ntx * d_scnum;
        memcpy(&out[0], &in[0], sizeof(gr_complex) * d_data_symbols * d_scnum);
    }

    toffset = nitems_written(0);
    add_packet_tag(toffset, noutput_syms * d_scnum);

    return noutput_syms * d_scnum;
}

void
mu_chanest_impl::add_frame_tag(uint64_t offset)
{
    for (int k = 0; k < d_nrx; k++)
        add_item_tag(k, offset,
                     pmt::string_to_symbol("packet_start"),
                     pmt::from_uint64(offset), _id);
}

void
mu_chanest_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int k = 0; k < d_nrx; k++)
        add_item_tag(k, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len), _id);
}

void
mu_chanest_impl::add_power_noise_tag(uint64_t offset, float signal_pwr, float noise_est)
{
    add_item_tag(0, offset,
                 pmt::string_to_symbol("noise_var"),
                 pmt::from_float(noise_est), _id);

    add_item_tag(0, offset + 1,
                 pmt::string_to_symbol("signal_pwr"),
                 pmt::from_float(signal_pwr), _id);
}

void
mu_chanest_impl::add_cpe_est_tag(uint64_t offset, float cpe_phi1, float cpe_offset1, float cpe_phi2, float cpe_offset2)
{
    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_phi1"),
                 pmt::from_float(cpe_phi1), _id);

    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_phi2"),
                 pmt::from_float(cpe_phi2), _id);

    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_offset1"),
                 pmt::from_float(cpe_offset1), _id);

    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_offset2"),
                 pmt::from_float(cpe_offset2), _id);
}

void
mu_chanest_impl::send_cpe_message()
{
    unsigned char buf[8];
    auto const *cpephi = reinterpret_cast<unsigned char const *>(&d_cpe_phi1);
    for (int k = 0; k < d_ncpt; k++)
    {
        buf[k] = cpephi[k];
    }
    pmt::pmt_t dict = pmt::make_dict();
    dict = pmt::dict_add(dict, pmt::mp("cpe_comp"), pmt::PMT_T);
    pmt::pmt_t pdu = pmt::make_blob(buf, d_ncpt);
    message_port_pub(pmt::mp("cpe"), pmt::cons(dict, pdu));
}

/*void
mu_chanest_impl::send_snr_message()
{
    // convert floats to bytes (little endian format)
    unsigned char buf[8];
    auto const *sigpwr = reinterpret_cast<unsigned char const *>(&d_sigpwr_est);
    auto const *noise = reinterpret_cast<unsigned char const *>(&d_noise_est);
    for (int k = 0; k < 4; k++)
    {
        buf[k] = sigpwr[k];
        buf[k + 4] = noise[k];
    }
    // make and send PDU message
    pmt::pmt_t dict = pmt::make_dict();
    dict = pmt::dict_add(dict, pmt::mp("csi"), pmt::PMT_T);
    pmt::pmt_t pdu = pmt::make_blob(buf, 8);
    message_port_pub(pmt::mp("csi"), pmt::cons(dict, pdu));
}*/

/*void
mu_chanest_impl::send_csi_message()
{
    // make and send PDU message
    pmt::pmt_t dict = pmt::make_dict();
    if (d_reset_frames > 0)
        dict = pmt::dict_add(dict, pmt::mp("reset"), pmt::PMT_T);
    else
        dict = pmt::dict_add(dict, pmt::mp("csi"), pmt::PMT_T);
    pmt::pmt_t pdu = pmt::make_blob(&d_chan_csi[0], d_ntx * d_nrx * d_scnum * sizeof(gr_complex));
    message_port_pub(pmt::mp("csi"), pmt::cons(dict, pdu));
}*/

void
mu_chanest_impl::generate_cpt_pilots()
{
    // initial state for first data symbol
    d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x70; // offset = 3 or 4

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

    for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
        // update pilot parity for current symbol
        unsigned parity_bit = (d_pilot_lsfr >> 6) ^ ((d_pilot_lsfr >> 3) & 0x1);
        d_pilot_lsfr = ((d_pilot_lsfr << 1) & 0x7E) | parity_bit; // circular 7-bit shifter
        float pilot_parity = (parity_bit == 1) ? -1 : 1;

        // generate current pilots
        for (int i = 0; i < d_ncpt; i++) // for all pilot positions
        {
            unsigned int idx = (symidx + i) % d_ncpt;
            for (int k = 0; k < d_ntx; k++)  // for all streams
            {
                int ueidx = k / d_nss;  // UE index
                int sidx = k % 2; // FIXME n_nrx // (d_nue * d_nss);  // stream index per total streams across all users
                int offset = d_ntx * d_ncpt * symidx + k * d_ncpt + i;
                if (d_mucpt && (symidx % d_nue) != ueidx)
                    d_cpt_pilot[offset] = 0;
                else
                    d_cpt_pilot[offset] = pilot_parity * basePilot[sidx][idx];
            }
        }
    }
}

void
mu_chanest_impl::cpe_estimate_comp(gr_vector_const_void_star &input_items,
                                   gr_vector_void_star &output_items,
                                   int noutsymcnt)
{
    float cpe_est_mean_0 = 0.0;
    float cpe_est_mean_1 = 0.0;
    for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
        unsigned input_offset = (d_ntx + symidx) * d_scnum;
        gr_complex cpe_sum = 0;

        for (int i = 0; i < d_ncpt; i++)  // loop for all pilots
        {
            unsigned pidx = d_cpt_idx[i];
            for (int k = 0; k < d_nrx; k++)  // loop for all receiver antennas
            {
                int pilot_offset = d_nrx * d_ncpt * symidx + k * d_ncpt + i;
                d_est_pilots[pilot_offset] = 0;  // estimate received pilots
                for (int n = 0; n < d_ntx; n++)
                { // combining all Tx streams
                    int offset = d_ntx * d_ncpt * symidx + n * d_ncpt + i;
                    // row major layout: (num_sc, nrx, ntx)
                    // d_est_pilots[pilot_offset] += d_chan_est[d_ntx * d_nrx * pidx + d_ntx * k + n] * d_cpt_pilot[offset];
                    // row major layout: (num_sc, ntx, nrx)
                    d_est_pilots[pilot_offset] +=
                        d_chan_est[d_ntx * d_nrx * pidx + d_nrx * n + k] * d_cpt_pilot[offset];
                }
                // sum over all pilots and receiver antennas
                auto *in = (const gr_complex *) input_items[k];
                cpe_sum += in[input_offset + pidx] * std::conj(d_est_pilots[pilot_offset]);
            }
        }
        d_cpe_est[symidx] = -1 * std::arg(cpe_sum);
        if (symidx % 2 == 0)
            cpe_est_mean_0 += d_cpe_est[symidx];
        else
            cpe_est_mean_1 += d_cpe_est[symidx];
    }

    // linear regression for CPE estimation
    if (!d_mucpt)
    {
        float cpe_est_mean = (cpe_est_mean_0 + cpe_est_mean_1) / (float) d_data_symbols;
        float x_mean = float(d_data_symbols - 1) / 2.0f;
        float x_sum = 0.0f, y_sum = 0.0f;
        for (int n = 0; n < d_data_symbols; n++)
        {
            float dx = (float) n - x_mean;
            float dy = d_cpe_est[n] - cpe_est_mean;
            y_sum += dx * dy;
            x_sum += dx * dx;
        }
        d_cpe_phi1 = y_sum / x_sum;
        d_cpe_offset1 = cpe_est_mean - d_cpe_phi1 * x_mean;
        d_cpe_phi2 = d_cpe_phi1;
        d_cpe_offset2 = d_cpe_offset1;
        if (d_total_frames % d_logfreq == 0)
            dout << "CPE estimate: (" << d_cpe_phi1 << ", " << d_cpe_offset1 << ")" << std::endl;
    }
    else
    {
        cpe_est_mean_0 /= (float) (d_data_symbols / 2);
        float x_mean = float(d_data_symbols + (d_data_symbols % 2) - 2) / 2.0f;
        float x_sum = 0.0f, y_sum = 0.0f;
        for (int n = 0; n < d_data_symbols; n += 2)
        {
            float dx = (float) n - x_mean;
            float dy = d_cpe_est[n] - cpe_est_mean_0;
            y_sum += dx * dy;
            x_sum += dx * dx;
        }
        d_cpe_phi1 = y_sum / x_sum;
        d_cpe_offset1 = cpe_est_mean_0 - d_cpe_phi1 * x_mean;
        if (d_total_frames % d_logfreq == 0)
            dout << "CPE estimate 1: (" << d_cpe_phi1 << ", " << d_cpe_offset1 << ")" << std::endl;

        cpe_est_mean_1 /= (float) (d_data_symbols / 2);
        x_sum = 0.0f, y_sum = 0.0f;
        for (int n = 1; n < d_data_symbols; n += 2)
        {
            float dx = (float) n - x_mean;
            float dy = d_cpe_est[n] - cpe_est_mean_1;
            y_sum += dx * dy;
            x_sum += dx * dx;
        }
        d_cpe_phi2 = y_sum / x_sum;
        d_cpe_offset2 = cpe_est_mean_1 - d_cpe_phi2 * x_mean;
        if (d_total_frames % d_logfreq == 0)
            dout << "CPE estimate 2: (" << d_cpe_phi2 << ", " << d_cpe_offset2 << ")" << std::endl;
    }
    // send_cpe_message();

    d_sigpwr_sum = 0.0;
    d_noise_sum = 0.0;
    for (int symidx = 0; symidx < d_data_symbols; symidx += 2)
    {
        unsigned input_offset = (d_ntx + symidx) * d_scnum;

        // CPE compensation for estimated pilots
        float cpe_phi = d_cpe_offset1 + symidx * d_cpe_phi1;
        gr_complex pilot_comp = std::exp(gr_complex(0, -cpe_phi));

        // estimate noise and signal power
        double power_est = 0, noise_est = 0;
        for (int i = 0; i < d_ncpt; i++)
        {
            for (int m = 0; m < d_nrx; m++)
            {
                auto *in = (const gr_complex *) input_items[m];
                int pilot_offset = d_nrx * d_ncpt * symidx + m * d_ncpt + i;
                d_est_pilots[pilot_offset] *= pilot_comp;
                power_est += std::norm(d_est_pilots[pilot_offset]);
                noise_est += std::norm(d_est_pilots[pilot_offset] - in[input_offset + d_cpt_idx[i]]);
            }
        }
        d_sigpwr_sum += power_est / (d_ncpt * d_nrx);
        d_noise_sum += noise_est / (d_ncpt * d_nrx);
    }
    for (int symidx = 1; symidx < d_data_symbols; symidx += 2)
    {
        unsigned input_offset = (d_ntx + symidx) * d_scnum;

        // CPE compensation for estimated pilots
        float cpe_phi = d_cpe_offset2 + symidx * d_cpe_phi2;
        gr_complex pilot_comp = std::exp(gr_complex(0, -cpe_phi));

        // estimate noise and signal power
        double power_est = 0, noise_est = 0;
        for (int i = 0; i < d_ncpt; i++)
        {
            for (int m = 0; m < d_nrx; m++)
            {
                auto *in = (const gr_complex *) input_items[m];
                int pilot_offset = d_nrx * d_ncpt * symidx + m * d_ncpt + i;
                d_est_pilots[pilot_offset] *= pilot_comp;
                power_est += std::norm(d_est_pilots[pilot_offset]);
                noise_est += std::norm(d_est_pilots[pilot_offset] - in[input_offset + d_cpt_idx[i]]);
            }
        }
        d_sigpwr_sum += power_est / (d_ncpt * d_nrx);
        d_noise_sum += noise_est / (d_ncpt * d_nrx);
    }
}

void
mu_chanest_impl::ltf_chan_est_1tx(gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items,
                                  int output_offset)
{
    // const float normfactor = sqrt(float(d_scnum) / float(d_fftsize));
    const float normfactor = sqrt(1.0f / float(d_nss));

    for (int n = 0; n < d_nue; n++)
    {
        // LTF for n-th UE is in n-th block in n-th receive antenna block
        const int input_offset = d_scnum * (d_preamble_symbols - (d_nue - n) * d_nss);
        int sidx = n * d_nss; // stream index for all UEs, 1 stream per UE

        for (int m = 0; m < d_nrx; m++)  // for all rx antennas
        {
            auto *in = (const gr_complex *) input_items[m] + input_offset;
            auto *out = (gr_complex *) output_items[m]; // all streams for rx
            for (int cidx = 0; cidx < d_scnum; cidx++) // carrier index
            {
                auto hest = normfactor * NORM_LTF_SEQ[cidx] * in[cidx];
                if (d_remove_cyclic_shift && (sidx % d_ntx) != 0)   // remove cyclic shift
                {
                    int offset = d_scnum + cidx; // shift index
                    hest *= d_cshift[offset];
                }
                // row major layout: (num_sc, nrx, ntx)
                // int caddr = d_ntx * d_nrx * cidx + d_ntx * m + sidx;
                // row major layout: (num_sc, ntx, nrx)
                int caddr = d_ntx * d_nrx * cidx + d_nrx * sidx + m;
                d_chan_est[caddr] = hest;
                // out[output_offset + d_nrx * cidx + m] = hest; // separate stream in each port
                // row major layout: (nss, num_sc)
                out[output_offset + d_ntx * cidx + sidx] = hest;
            }
        }
    }
}

void
mu_chanest_impl::ltf_chan_est_2tx(gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items,
                                  int output_offset)
{
    // const float normfactor = sqrt(float(d_scnum) / float(d_nss * d_fftsize));
    const float normfactor = 0.5f; // (1.0/nss)
    CMatrix2 Pd, Rx;
    Pd << normfactor, -normfactor, normfactor, normfactor;

    // LTF for n-th UE is in n-th block in n-th receive antenna block
    const int input_offset = d_scnum * (d_preamble_symbols - d_nue * d_nss);

    // channel estimation for 2 rx antennas
    auto *in0 = (const gr_complex *) input_items[0] + input_offset;
    auto *in1 = (const gr_complex *) input_items[1] + input_offset;
    auto *out0 = (gr_complex *) output_items[0];
    auto *out1 = (gr_complex *) output_items[1];
    for (int cidx = 0; cidx < d_scnum; cidx++) // subcarrier index
    {
        Rx(0, 0) = in0[cidx], Rx(1, 0) = in0[cidx + d_scnum];  // (s0,r0), (s1,r0)
        Rx(0, 1) = in1[cidx], Rx(1, 1) = in1[cidx + d_scnum];  // (s0,r1) (s1,r1)
        // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        CMatrix2 H1 = NORM_LTF_SEQ[cidx] * Pd * Rx;
        Rx(0, 0) = in0[cidx + 2 * d_scnum], Rx(1, 0) = in0[cidx + 3 * d_scnum];  // (s0,r0), (s1,r0)
        Rx(0, 1) = in1[cidx + 2 * d_scnum], Rx(1, 1) = in1[cidx + 3 * d_scnum];  // (s0,r1) (s1,r1)
        CMatrix2 H2 = NORM_LTF_SEQ[cidx] * Pd * Rx;

        // Eigen::Map<CMatrix2> H(&d_chan_est[d_nrx * d_ntx * cidx], d_ntx, d_nrx);
        // H << H1, H2;
        using CMatrixXr = Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

        size_t base = static_cast<size_t>(d_ntx) * d_nrx * cidx;
        Eigen::Map<CMatrixXr> H(&d_chan_est[base], d_ntx, d_nrx);  // maps 4×2 at runtime

        H.block<2,2>(0,0) = H1;
        H.block<2,2>(2,0) = H2;

        // remove cyclic shift
        if (d_remove_cyclic_shift)
            for (int s = 1; s < d_ntx; s++) // for all except 1st stream
            {
                int sidx = (s % 2) * d_scnum + cidx; // shift index
                for (int n = 0; n < d_nrx; n++) // for receive
                    H(s, n) *= d_cshift[sidx];
            }

        // copy channel estimation for all streams per receiver antennas to individual output port
        // using column major memory layout for Eigen
        int chanest_offset = output_offset + d_ntx * cidx;
        out0[chanest_offset] = H(0, 0);  // s0,r0
        out0[chanest_offset + 1] = H(1, 0);  // s1,r0
        out0[chanest_offset + 2] = H(2, 0);  // s2,r0
        out0[chanest_offset + 3] = H(3, 0);  // s3,r0
        out1[chanest_offset] = H(0, 1);  // s0,r1
        out1[chanest_offset + 1] = H(1, 1); // s1,r1
        out1[chanest_offset + 2] = H(2, 1);  // s2,r1
        out1[chanest_offset + 3] = H(3, 1); // s3,r1
    }
}

void
mu_chanest_impl::ltf_chan_est_2rx(gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items,
                                  int output_offset)
{
    auto *in0 = (const gr_complex *) input_items[0] + d_scnum * (d_preamble_symbols - d_nss);
    auto *in1 = (const gr_complex *) input_items[1] + d_scnum * (d_preamble_symbols - d_nss);
    auto *out0 = (gr_complex *) output_items[0];
    auto *out1 = (gr_complex *) output_items[1];

    CMatrix2 Pd, Rx;
    const float normfactor = 1.0f / float(d_nss);
    Pd << normfactor, -normfactor, normfactor, normfactor;
    for (int cidx = 0; cidx < d_scnum; cidx++)
    {
        Rx(0, 0) = in0[cidx], Rx(1, 0) = in0[cidx + d_scnum];  // (s0,r0), (s1,r0)
        Rx(0, 1) = in1[cidx], Rx(1, 1) = in1[cidx + d_scnum];  // (s0,r1) (s1,r1)
        // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        Eigen::Map<CMatrix2> H(&d_chan_est[4 * cidx], 2, 2);
        H = NORM_LTF_SEQ[cidx] * Pd * Rx;

        // remove cyclic shift
        if (d_remove_cyclic_shift)
            for (int s = 1; s < d_ntx; s++) // for all except 1st stream
            {
                int sidx = (s % 2) * d_scnum + cidx; // shift index
                for (int n = 0; n < d_nrx; n++) // for receive
                    H(s, n) *= d_cshift[sidx];
            }

        // copy channel estimation for all streams per receiver antennas to individual output port
        // (s0,r0),(s1,r0) -> ch0,  (s0,r1),(s1,r1) -> ch1
        // using column-major memory layout
        out0[output_offset + 2 * cidx] = H(0, 0);  // s0,r0
        out0[output_offset + 2 * cidx + 1] = H(1, 0);  // s1,r0
        out1[output_offset + 2 * cidx] = H(0, 1);  // s0,r1
        out1[output_offset + 2 * cidx + 1] = H(1, 1); // s1,r1
    }
}

void
mu_chanest_impl::ltf_chan_est_nrx(gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items,
                                  int output_offset)
{
    CMatrixX Rx(d_nss, d_nrx);  // nLTF = nSTS for normal full multiplexing
    for (int k = 0; k < d_scnum; k++)  // for all subcarriers
    {
        // copy LTF inputs
        for (int m = 0; m < d_nrx; m++)  // for all rx
        {
            auto in = (const gr_complex *) input_items[m] + d_scnum * (d_preamble_symbols - d_nss);
            for (int n = 0; n < d_nss; n++)  // for all LTFs
                Rx(n, m) = in[n * d_scnum + k];
        }
        // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        Eigen::Map<CMatrixX> H(&d_chan_est[d_nss * d_nrx * k], d_nss, d_nrx);
        H = NORM_LTF_SEQ[k] * d_Pd * Rx;
        // copy channel estimation for each stream to individual output port
        // support multiple rx antennas sets (nRx = k * nSTS)
        for (int m = 0; m < d_nrx; m++) // for all streams
        {
            auto out = (gr_complex *) output_items[m];
            int ri = (m / d_nss) * d_nss; // receiver antennas base index
            for (int n = 0; n < d_nss; n++) // for all rx antennas in a set
                out[output_offset + d_nss * k + n] = H(m % d_nss, ri + n);
        }
    }
}

const gr_vector_float mu_chanest_impl::NORM_LTF_SEQ_64 = {
    1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1,
    1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1,
    -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1};

const gr_vector_float mu_chanest_impl::NORM_LTF_SEQ_256A = {
    -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1,
    -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1,
    -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1,
    1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1,
    1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1};

const gr_vector_float mu_chanest_impl::NORM_LTF_SEQ_256B = {
    -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1,
    -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1,
    -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1,
    1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1,
    -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1};

} /* namespace gr::ncjt */
