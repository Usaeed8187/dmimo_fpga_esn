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
mu_chanest::make(int fftsize, int ntx, int nrx, int nue, int npreamblesyms, int ndatasyms,
                 int logfreq, bool debug)
{
    return gnuradio::make_block_sptr<mu_chanest_impl>(
        fftsize, ntx, nrx, nue, npreamblesyms, ndatasyms, logfreq, debug);
}

mu_chanest_impl::mu_chanest_impl(int fftsize, int ntx, int nrx, int nue,
                                 int npreamblesyms, int ndatasyms, int logfreq, bool debug)
    : gr::tagged_stream_block(
    "mu_chanest",
    gr::io_signature::make(1, 8, sizeof(gr_complex)),
    gr::io_signature::make(1, 8, sizeof(gr_complex)),
    "packet_len"),
      d_preamble_symbols(npreamblesyms), d_data_symbols(ndatasyms),
      d_total_frames(0), d_reset_frames(0), d_logfreq(logfreq), d_debug(debug)
{
    if (fftsize == 64)
    {
        d_scnum = 56;
        d_npt = 4;
        NORM_LTF_SEQ = NORM_LTF_SEQ_64;
    }
    else if (fftsize == 256)
    {
        d_scnum = 242;
        d_npt = 8;
        NORM_LTF_SEQ = NORM_LTF_SEQ_256;
    }
    else
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;

    if (ntx < 1 || ntx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Tx antennas");
    d_nrx = nrx;
    if (nrx < 1 || nrx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Rx antennas");
    d_ntx = ntx;

    if (nue < 1 || nue > ntx || nue > nrx || ntx % nue != 0)
        throw std::runtime_error("invalid number of UEs specified");
    d_nue = nue;

    d_nss = d_ntx / d_nue;  // number of streams per UE
    if (d_nss != 1 && d_nss != 2 && d_nss != 4)
        throw std::runtime_error("currently only support 1, 2 or 4 multiplexing streams");

    d_cur_sym = -npreamblesyms;
    d_last_sym = d_cur_sym;

    d_chan_est = malloc_complex(d_ntx * d_nrx * d_scnum);
    d_cshift = malloc_complex(4 * d_scnum);

    // cyclic shift compensation
    // TODO: fft_size = 256
    float cshift[4] = {0, -8, -4, -12};
    for (int m = 0; m < 4; m++)
    {
        for (int k = 0; k < d_scnum / 2; k++)
            d_cshift[m * d_scnum + k] = exp(gr_complex(0, 2.0 * M_PI / d_fftsize * cshift[m] * (k - 28)));
        for (int k = d_scnum / 2; k < d_scnum; k++)
            d_cshift[m * d_scnum + k] = exp(gr_complex(0, 2.0 * M_PI / d_fftsize * cshift[m] * (k - 27)));
    }

    // Pd: nSTS x nLTF
    if (d_nss == 2) // 2Tx2Rx, 2Tx4Rx
    {
        d_Pd.resize(2, 2);
        // d_Pd << 1, -1, 1, 1; // column major
        d_Pd << 1, 1, -1, 1; // row-major
    }
    else if (d_nss == 4) // 4Tx4Rx
    {
        d_Pd.resize(4, 4);
        // d_Pd << 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1; // column major
        d_Pd << 1, 1, 1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1; // row major
    }

    const float normfactor = sqrt(d_scnum / float(d_ntx * d_fftsize));
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
    if (ninput_syms <= 0)
        return 0;

    int noutput_syms = 0;
    int nsymcnt = 0;
    for (; nsymcnt < ninput_syms; nsymcnt++)
    {
        if (d_cur_sym == -1) // full HT-LTFs available
        {
            update_pilots(d_cur_sym);
            if (d_nss == 1) // d_nue >= 1
                ltf_chan_est_1tx(input_items, output_items, noutput_syms * d_scnum);
            else if (d_nss == 2 && d_nue > 1)
                ltf_chan_est_2tx(input_items, output_items, noutput_syms * d_scnum);
            else if (d_nss == 2 && d_nrx == 2)
                ltf_chan_est_2rx(input_items, output_items, noutput_syms * d_scnum);
            else
                ltf_chan_est_nrx(input_items, output_items, noutput_syms * d_scnum);
            uint64_t offset = nitems_written(0) + noutput_syms * d_scnum;
            // add_packet_tag(offset, d_ntx * d_scnum);
            add_packet_tag(offset, (d_ntx + d_data_symbols) * d_scnum);
            add_frame_tag(offset);
            noutput_syms += d_ntx; // d_nue * d_nss;
            d_sigpwr_sum = 0.0;
            d_noise_sum = 0.0;
        }

        // add packet_len tags for tagged stream block
        /*if (d_cur_sym >= 0 && d_cur_sym % 5 == 0)
        {
            auto offset = nitems_written(0) + noutput_syms * d_scnum;
            int packet_len = (d_cur_sym < d_data_symbols - 8) ? 5 * d_scnum : (d_data_symbols - d_last_sym) * d_scnum;
            add_packet_tag(offset, packet_len);
            // dout << "Packet from symbol " << (d_cur_sym + 1) << ", length " << packet_len <<
            //     " offset " << offset << std::endl;
        }*/

        if (d_cur_sym >= 0)
        {
            update_pilots(d_cur_sym);
            cpe_estimate_comp(input_items, output_items, nsymcnt, noutput_syms);
            send_cpe_message();
            noutput_syms += 1;
        }

        d_cur_sym += 1;

        if (d_cur_sym > 0 && (d_cur_sym % 5 == 0 || d_cur_sym == d_data_symbols))
        {
            d_sigpwr_est = (float) d_sigpwr_sum / (float) d_cur_sym;
            d_noise_est = (float) d_noise_sum / (float) d_cur_sym;
            d_last_sym = d_cur_sym;

            auto offset = nitems_written(0) + (noutput_syms - 1) * d_scnum;
            add_power_noise_tag(offset, d_sigpwr_est, d_noise_est);
            // send_snr_message();
        }

        if (d_cur_sym == d_data_symbols)
        {
            d_total_frames += 1;
            if (d_reset_frames > 0)
                d_reset_frames -= 1;
            d_cur_sym = -d_preamble_symbols;
            d_sigpwr_sum = 0.0;
            d_noise_sum = 0.0;
            auto snr_db = 10.0 * std::log10(d_sigpwr_est / d_noise_est);
            if (d_total_frames % d_logfreq == 0)
                dout << ">>>>>>>> Signal power: " << d_sigpwr_est << "  noise: " << d_noise_est
                     << " (SNR " << snr_db << " dB)" << std::endl;
        }
    }

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
    for (int k = 0; k < d_nrx; k += 2)
    {
        add_item_tag(k, offset,
                     pmt::string_to_symbol("signal_pwr"),
                     pmt::from_float(signal_pwr), _id);

        add_item_tag(k + 1, offset,
                     pmt::string_to_symbol("noise_var"),
                     pmt::from_float(noise_est), _id);
    }
}

void
mu_chanest_impl::send_cpe_message()
{
    unsigned char buf[8];
    auto const *cpephi = reinterpret_cast<unsigned char const *>(&d_cpe_phi);
    for (int k = 0; k < d_npt; k++)
    {
        buf[k] = cpephi[k];
    }
    pmt::pmt_t dict = pmt::make_dict();
    dict = pmt::dict_add(dict, pmt::mp("cpe_comp"), pmt::PMT_T);
    pmt::pmt_t pdu = pmt::make_blob(buf, d_npt);
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
mu_chanest_impl::update_pilots(int symidx)
{
    // initial state for first data symbol
    if (symidx < 0)
    {
        d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x70; // TODO: check offset values
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

    auto basePilot = (d_fftsize == 64) ? ((d_ntx == 2) ? basePilots4 : basePilots4 + 2) : basePilots8;

    // generate current pilots
    for (int i = 0; i < d_npt; i++) // for all pilot positions
    {
        unsigned int idx = (symidx + i) % d_npt;
        for (int k = 0; k < d_ntx; k++)  // for all streams
            d_cur_pilot[k][i] = pilot_parity * basePilot[k][idx];
    }
}

void
mu_chanest_impl::cpe_estimate_comp(gr_vector_const_void_star &input_items,
                                   gr_vector_void_star &output_items,
                                   int nsymcnt,
                                   int noutsymcnt)
{
    unsigned input_offset = nsymcnt * d_scnum;
    unsigned output_offset = noutsymcnt * d_scnum;

    const unsigned pilot_idx4[8] = {7, 21, 34, 48};
    const unsigned pilot_idx8[8] = {6, 32, 74, 100, 141, 167, 209, 235};
    auto pilot_idx = (d_fftsize == 64) ? pilot_idx4 : pilot_idx8;
    gr_complex est_pilots[MAX_NSS][8]; // estimated pilots
    gr_complex cpe_sum = 0;

    for (int i = 0; i < d_npt; i++)  // loop for all pilots
    {
        unsigned pidx = pilot_idx[i];
        for (int k = 0; k < d_nrx; k++)  // loop for all receiver antennas
        {
            est_pilots[k][i] = 0;  // estimate received pilots
            for (int n = 0; n < d_ntx; n++)  // combining all Tx streams
                // est_pilots[k][i] += d_chan_est[d_ntx * d_nrx * pidx + d_ntx * k + n] * d_cur_pilot[n][i];
                est_pilots[k][i] += d_chan_est[d_ntx * d_nrx * pidx + d_ntx * n + k] * d_cur_pilot[n][i];
            // sum over all pilots and receiver antennas
            auto *in = (const gr_complex *) input_items[k];
            cpe_sum += in[input_offset + pidx] * std::conj(est_pilots[k][i]);
        }
    }
    d_cpe_phi = -1 * std::arg(cpe_sum);
    // dout << "CPE estimate: " << d_cpe_phi << std::endl;

    // CPE compensation for received pilots and data
    const float normfactor = sqrt(float(d_ntx * d_scnum) / d_fftsize);
    gr_complex cpe_comp = normfactor * std::exp(gr_complex(0, d_cpe_phi));
    for (int m = 0; m < d_nrx; m++)
    {
        auto *in = (const gr_complex *) input_items[m];
        auto *out = (gr_complex *) output_items[m];
        volk_32fc_s32fc_multiply2_32fc(&out[output_offset], &in[input_offset], &cpe_comp, d_scnum);
    }

    // estimate noise and signal power
    double power_est = 0, noise_est = 0;
    for (int i = 0; i < d_npt; i++)
    {
        for (int m = 0; m < d_nrx; m++)
        {
            auto *out = (gr_complex *) output_items[m];
            power_est += std::norm(est_pilots[m][i]);
            noise_est += std::norm(est_pilots[m][i] - out[output_offset + pilot_idx[i]]);
        }
    }
    d_sigpwr_sum += power_est / (d_npt * d_nrx);
    d_noise_sum += noise_est / (d_npt * d_nrx);
}

void
mu_chanest_impl::ltf_chan_est_1tx(gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items,
                                  int output_offset)
{
    const float normfactor = sqrt(float(d_scnum) / float(d_fftsize));

    for (int n = 0; n < d_nue; n++)
    {
        // LTF for n-th UE is in n-th block in n-th receive antenna block
        const int input_offset = d_scnum * (d_preamble_symbols - (d_nue - n) * d_nss);
        int sidx = n * d_nss; // stream index, 1 stream per UE

        for (int m = 0; m < d_nrx; m++)  // for all rx antennas
        {
            auto *in = (const gr_complex *) input_items[m] + input_offset;
            // auto *out = (gr_complex *) output_items[sidx]; // separate stream in each port
            auto *out = (gr_complex *) output_items[m];
            for (int cidx = 0; cidx < d_scnum; cidx++) // carrier index
            {
                auto hest = normfactor * NORM_LTF_SEQ[cidx] * in[cidx];
                // row major layout: (num_sc, nrx, nss)
                int caddr = d_ntx * d_nrx * cidx + d_ntx * sidx + m; // 1 stream only
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
    const float normfactor = sqrt(float(d_scnum) / float(d_nss * d_fftsize));
    CMatrix2 Pd, Rx;
    Pd << normfactor, -normfactor, normfactor, normfactor;

    for (int uidx = 0; uidx < d_nue; uidx++)
    {
        // LTF for n-th UE is in n-th block in n-th receive antenna block
        const int input_offset = d_scnum * (d_preamble_symbols - (d_nue - uidx) * d_nss);

        for (int m = 0; m < d_nrx; m += d_nss)  // for all rx antennas
        {
            // channel estimation for 2 rx antennas
            auto *in0 = (const gr_complex *) input_items[m] + input_offset;
            auto *in1 = (const gr_complex *) input_items[m + 1] + input_offset;
            auto *out0 = (gr_complex *) output_items[m];
            auto *out1 = (gr_complex *) output_items[m + 1];
            for (int cidx = 0; cidx < d_scnum; cidx++) // subcarrier index
            {
                Rx(0, 0) = in0[cidx], Rx(1, 0) = in0[cidx + d_scnum];  // (s0,r0), (s1,r0)
                Rx(0, 1) = in1[cidx], Rx(1, 1) = in1[cidx + d_scnum];  // (s0,r1) (s1,r1)
                // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
                // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
                Eigen::Map<CMatrix2> H(&d_chan_est[d_nss * d_ntx * cidx], 2, 2);
                H = NORM_LTF_SEQ[cidx] * Pd * Rx;
                // copy channel estimation for all streams per receiver antennas to individual output port
                // (s0,r0),(s1,r0) -> ch0,  (s0,r1),(s1,r1) -> ch1
                // using column-major memory layout, shape ()
                int chanest_offset = output_offset +  d_ntx * cidx + d_nss * uidx;
                out0[chanest_offset] = H(0, 0);  // s0,r0
                out0[chanest_offset + 1] = H(1, 0);  // s1,r0
                out1[chanest_offset] = H(0, 1);  // s0,r1
                out1[chanest_offset + 1] = H(1, 1); // s1,r1
            }
        }
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
    const float normfactor = sqrt(d_scnum / float(d_nss * d_fftsize));
    Pd << normfactor, -normfactor, normfactor, normfactor;
    for (int k = 0; k < d_scnum; k++)
    {
        Rx(0, 0) = in0[k], Rx(1, 0) = in0[k + d_scnum];  // (s0,r0), (s1,r0)
        Rx(0, 1) = in1[k], Rx(1, 1) = in1[k + d_scnum];  // (s0,r1) (s1,r1)
        // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        Eigen::Map<CMatrix2> H(&d_chan_est[4 * k], 2, 2);
        H = NORM_LTF_SEQ[k] * Pd * Rx;
        // copy channel estimation for all streams per receiver antennas to individual output port
        // (s0,r0),(s1,r0) -> ch0,  (s0,r1),(s1,r1) -> ch1
        // using column-major memory layout
        out0[output_offset + 2 * k] = H(0, 0);  // s0,r0
        out0[output_offset + 2 * k + 1] = H(1, 0);  // s1,r0
        out1[output_offset + 2 * k] = H(0, 1);  // s0,r1
        out1[output_offset + 2 * k + 1] = H(1, 1); // s1,r1
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

const gr_vector_float mu_chanest_impl::NORM_LTF_SEQ_256 = {
    -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1,
    -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1,
    -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1,
    1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1,
    1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1};

} /* namespace gr::ncjt */
