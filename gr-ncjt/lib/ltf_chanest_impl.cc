/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "ltf_chanest_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "utils.h"

namespace gr::ncjt
{

ltf_chanest::sptr
ltf_chanest::make(int fftsize, int ntx, int nrx, int npreamblesyms, int ndatasyms,
                  bool csifb, bool removecs, int logfreq, bool debug)
{
    return gnuradio::make_block_sptr<ltf_chanest_impl>(
        fftsize, ntx, nrx, npreamblesyms, ndatasyms, csifb, removecs, logfreq, debug);
}

ltf_chanest_impl::ltf_chanest_impl(int fftsize, int ntx, int nrx, int npreamblesyms, int ndatasyms,
                                   bool csifb, bool removecs, int logfreq, bool debug)
    : gr::tagged_stream_block("ltf_chanest",
                              gr::io_signature::make(nrx, nrx, sizeof(gr_complex)),
                              gr::io_signature::make(nrx, nrx, sizeof(gr_complex)),
                              "packet_len"),
      d_preamble_symbols(npreamblesyms), d_data_symbols(ndatasyms), d_csi_en(csifb), d_remove_cyclic_shift(removecs),
      d_total_frames(0), d_reset_frames(0), d_logfreq(logfreq), d_debug(debug)
{
    if (fftsize != 64 && fftsize != 256)
        throw std::runtime_error("Unsupported OFDM FFT size");
    d_fftsize = fftsize;

    if (d_fftsize == 64)
    {
        d_scnum = 56;
        d_npt = 4;
        NORM_LTF_SEQ = NORM_LTF_SEQ_64;
    }
    else
    {
        d_scnum = 242;
        d_npt = 8;
        NORM_LTF_SEQ = NORM_LTF_SEQ_256;
    }

    if (ntx < 1 || ntx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Tx antennas");
    d_nrx = nrx;
    if (nrx < 1 || nrx > MAX_NSS)
        throw std::runtime_error("only support 1 to 8 Rx antennas");
    d_ntx = ntx;
    d_nss = std::min(d_ntx, d_nrx); // number of spatial streams for multiplexing
    if (d_nss != 1 && d_nss != 2 && d_nss != 4)
        throw std::runtime_error("currently only support 1, 2 or 4 multiplexing streams");
    if (d_nrx < d_nss)
        throw std::runtime_error("number of Rx antennas must be greater than number of streams");

    d_cpt_pilot = malloc_float(d_npt * d_nss * d_data_symbols);
    d_cpe_est = malloc_float(d_data_symbols);
    d_est_pilots = malloc_complex(d_npt * d_nrx * d_data_symbols);
    d_chan_est = malloc_complex(d_nss * d_nrx * d_scnum);
    d_chan_csi = malloc_complex(d_ntx * d_nrx * d_scnum);
    d_cshift = malloc_complex(4 * d_scnum);

    // pre-generate all CPT pilots
    generate_cpt_pilots();

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
    if (d_ntx == 4 || d_nss == 4) // 4x4, 4x2
    {
        d_Pd.resize(4, 4);
        d_Pd << 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1;
        /*
        1, -1, 1, 1, 
        1, 1, -1, 1, 
        1, 1, 1, -1, 
        -1, 1, 1, 1
        */
    }
    else if (d_nss == 2) // 2x2, 2x4
    {
        d_Pd.resize(2, 2);
        d_Pd << 1, -1, 1, 1;
    }

    const float normfactor = 1.0f / float(d_nss);
    d_Pd = normfactor * d_Pd;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    message_port_register_in(pmt::mp("reset"));
    set_msg_handler(pmt::mp("reset"), [this](const pmt::pmt_t &msg) { process_reset_message(msg); });

    message_port_register_out(pmt::mp("cpe"));
    message_port_register_out(pmt::mp("csi"));

    set_tag_propagation_policy(block::TPP_DONT);
}

ltf_chanest_impl::~ltf_chanest_impl()
{
    if (d_cpt_pilot != nullptr)
        volk_free(d_cpt_pilot);
    if (d_cpe_est != nullptr)
        volk_free(d_cpe_est);
    if (d_est_pilots != nullptr)
        volk_free(d_est_pilots);
    if (d_chan_est != nullptr)
        volk_free(d_chan_est);
    if (d_chan_csi != nullptr)
        volk_free(d_chan_csi);
    if (d_cshift != nullptr)
        volk_free(d_cshift);
}

int
ltf_chanest_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    return (d_nss + d_data_symbols) * d_scnum;
}

int
ltf_chanest_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_nrx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    int ninput_syms = min_input_items / d_scnum;
    if (ninput_syms != d_preamble_symbols + d_data_symbols)
        throw std::runtime_error("total number of OFDM symbols to channel estimation not correct");

    int noutput_syms = d_nss + d_data_symbols;
    int cur_sym = -d_preamble_symbols;
    for (int nsymcnt = 0; nsymcnt < d_preamble_symbols; nsymcnt++)
    {
        if (cur_sym == -1) // full HT-LTFs available
        {
            if (d_nss == 1)
                ltf_chan_est_1tx(input_items, output_items, 0);
            else if (d_nss == 2 && d_nrx == 2)
                ltf_chan_est_2rx(input_items, output_items, 0);
            else
                ltf_chan_est_nrx(input_items, output_items, 0);
            uint64_t offset = nitems_written(0);
            add_frame_tag(offset);
        }
        else if (d_csi_en && d_total_frames >= 10)
        {
            if (d_ntx == 2 && cur_sym == -d_preamble_symbols + 1)  // 2Tx-2Rx, 2Tx-4Rx
            {
                csi_chan_est_nrx(input_items, (nsymcnt - 1) * d_scnum);
                send_csi_message();
            }
            else if (d_ntx == 4 && cur_sym == -d_preamble_symbols + 3)  // 4Tx-2Rx, 4Tx-4Rx
            {
                csi_chan_est_nrx(input_items, (nsymcnt - 3) * d_scnum);
                send_csi_message();
            }
        }
        cur_sym += 1;
    }

    // CPE estimation and compensation
    cpe_estimate_comp(input_items, output_items, d_nss);

    d_total_frames += 1;
    if (d_reset_frames > 0)
        d_reset_frames -= 1;

    d_sigpwr_est = (float) d_sigpwr_sum / (float) d_data_symbols;
    d_noise_est = (float) d_noise_sum / (float) d_data_symbols;
    auto offset = nitems_written(0) + (noutput_syms - 1) * d_scnum;
    add_power_noise_tag(offset, d_sigpwr_est, d_noise_est);
    add_cpe_est_tag(offset, d_cpe_phi, d_cpe_offset);

    auto snr_db = 10.0 * std::log10(d_sigpwr_est / d_noise_est);
    if (d_total_frames % d_logfreq == 0)
        dout << ">>>>>>>> Signal power: " << d_sigpwr_est << "  noise: " << d_noise_est
             << " (SNR " << snr_db << " dB)" << std::endl;

    // Copy received symbols to output
    for (int m = 0; m < d_nrx; m++)
    {
        auto *in = (const gr_complex *) input_items[m] + d_preamble_symbols * d_scnum;
        auto *out = (gr_complex *) output_items[m] + d_nss * d_scnum;
        memcpy(&out[0], &in[0], sizeof(gr_complex) * d_data_symbols * d_scnum);
    }

    offset = nitems_written(0);
    add_packet_tag(offset, noutput_syms * d_scnum);

    return noutput_syms * d_scnum;
}

void
ltf_chanest_impl::add_frame_tag(uint64_t offset)
{
    for (int k = 0; k < d_nrx; k++)
        add_item_tag(k, offset,
                     pmt::string_to_symbol("packet_start"),
                     pmt::from_uint64(offset), _id);
}

void
ltf_chanest_impl::add_packet_tag(uint64_t offset, int packet_len)
{
    for (int k = 0; k < d_nrx; k++)
        add_item_tag(k, offset,
                     pmt::string_to_symbol("packet_len"),
                     pmt::from_long(packet_len), _id);
}

void
ltf_chanest_impl::add_power_noise_tag(uint64_t offset, float signal_pwr, float noise_est)
{
    add_item_tag(0, offset,
                 pmt::string_to_symbol("noise_var"),
                 pmt::from_float(noise_est), _id);

    add_item_tag(0, offset + 1,
                 pmt::string_to_symbol("signal_pwr"),
                 pmt::from_float(signal_pwr), _id);
}

void
ltf_chanest_impl::add_cpe_est_tag(uint64_t offset, float cpe_phi, float cpe_offset)
{
    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_phi1"),
                 pmt::from_float(cpe_phi), _id);

    add_item_tag(0, offset,
                 pmt::string_to_symbol("cpe_phi2"),
                 pmt::from_float(cpe_phi), _id);

    add_item_tag(0, offset + 1,
                 pmt::string_to_symbol("cpe_offset1"),
                 pmt::from_float(cpe_offset), _id);

    add_item_tag(0, offset + 1,
                 pmt::string_to_symbol("cpe_offset2"),
                 pmt::from_float(cpe_offset), _id);
}

void
ltf_chanest_impl::process_reset_message(const pmt::pmt_t &msg)
{
    if (d_csi_en)
    {
        std::cout << ">>>>>>>> Resetting CSI matrix ..." << std::endl;
        d_reset_frames = 10;
        send_csi_message();
    }
}

void
ltf_chanest_impl::send_cpe_message()
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

void
ltf_chanest_impl::send_snr_message()
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
}

void
ltf_chanest_impl::send_csi_message()
{
    // make and send PDU message
    pmt::pmt_t dict = pmt::make_dict();
    if (d_reset_frames > 0)
        dict = pmt::dict_add(dict, pmt::mp("reset"), pmt::PMT_T);
    else
        dict = pmt::dict_add(dict, pmt::mp("csi"), pmt::PMT_T);
    pmt::pmt_t pdu = pmt::make_blob(&d_chan_csi[0], d_ntx * d_nrx * d_scnum * sizeof(gr_complex));
    message_port_pub(pmt::mp("csi"), pmt::cons(dict, pdu));
}

void
ltf_chanest_impl::generate_cpt_pilots()
{
    // initial state for first data symbol
    d_pilot_lsfr = (d_fftsize == 64) ? 0x78 : 0x71; // offset = 3 or 4

    for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
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

        auto basePilot = (d_fftsize == 64) ? basePilots4 : basePilots8;

        // generate current pilots
        for (int i = 0; i < d_npt; i++) // for all pilot positions
        {
            unsigned int idx = (symidx + i) % d_npt;
            for (int k = 0; k < d_nss; k++)  // for all streams
            {
                int offset = d_nss * d_npt * symidx + k * d_npt + i;
                d_cpt_pilot[offset] = pilot_parity * basePilot[k][idx];
            }
        }
    }
}

void
ltf_chanest_impl::cpe_estimate_comp(gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items,
                                    int noutsymcnt)
{
    const unsigned pilot_idx4[8] = {7, 21, 34, 48};
    const unsigned pilot_idx8[8] = {6, 32, 74, 100, 141, 167, 209, 235};
    auto pilot_idx = (d_fftsize == 64) ? pilot_idx4 : pilot_idx8;

    float cpe_est_mean = 0.0;
    for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
        int input_offset = (d_preamble_symbols + symidx) * d_scnum;
        gr_complex cpe_sum = 0;

        for (int i = 0; i < d_npt; i++)  // loop for all pilots
        {
            unsigned pidx = pilot_idx[i];
            for (int k = 0; k < d_nrx; k++)  // loop for all receiver antennas
            {
                int pilot_offset = d_nrx * d_npt * symidx + k * d_npt + i;
                d_est_pilots[pilot_offset] = 0;  // estimate received pilots
                for (int n = 0; n < d_nss; n++)  // combining all streams
                {
                    // est_pilots[k][i] += d_chan_est[d_nss * d_nrx * pidx + d_nss * k + n] * d_cpt_pilot[n][i];
                    int offset = d_nss * d_npt * symidx + n * d_npt + i;
                    d_est_pilots[pilot_offset] +=
                        d_chan_est[d_nss * d_nrx * pidx + d_nrx * n + k] * d_cpt_pilot[offset];
                }
                // sum over all pilots and receiver antennas
                auto *in = (const gr_complex *) input_items[k];
                cpe_sum += in[input_offset + pidx] * std::conj(d_est_pilots[pilot_offset]);
            }
        }
        d_cpe_est[symidx] = -1 * std::arg(cpe_sum);
        cpe_est_mean += d_cpe_est[symidx];
    }

    // linear regression for CPE estimation
    cpe_est_mean /= (float) d_data_symbols;
    float x_mean = (d_data_symbols - 1.0f) / 2.0f;
    float x_sum = 0.0f, y_sum = 0.0f;
    for (int n = 0; n < d_data_symbols; n++)
    {
        float dx = (float) n - x_mean;
        float dy = d_cpe_est[n] - cpe_est_mean;
        y_sum += dx * dy;
        x_sum += dx * dx;
    }
    d_cpe_phi = y_sum / x_sum;
    d_cpe_offset = cpe_est_mean - d_cpe_phi * x_mean;
    if (d_total_frames % d_logfreq == 0)
        dout << "CPE estimate: (" << d_cpe_phi << ", " << d_cpe_offset << ")" << std::endl;
    // send_cpe_message();

    d_sigpwr_sum = 0.0;
    d_noise_sum = 0.0;
    for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
        unsigned input_offset = (d_preamble_symbols + symidx) * d_scnum;

        // CPE compensation for estimated pilots
        float cpe_phi = d_cpe_offset + symidx * d_cpe_phi;
        gr_complex pilot_comp = std::exp(gr_complex(0, -cpe_phi));

        // estimate noise and signal power
        double power_est = 0, noise_est = 0;
        for (int i = 0; i < d_npt; i++)
        {
            for (int m = 0; m < d_nrx; m++)
            {
                auto *in = (const gr_complex *) input_items[m];
                int pilot_offset = d_nrx * d_npt * symidx + m * d_npt + i;
                d_est_pilots[pilot_offset] *= pilot_comp;
                power_est += std::norm(d_est_pilots[pilot_offset]);
                noise_est += std::norm(d_est_pilots[pilot_offset] - in[input_offset + pilot_idx[i]]);
            }
        }
        d_sigpwr_sum += power_est / (d_npt * d_nrx);
        d_noise_sum += noise_est / (d_npt * d_nrx);
    }

    // CPE compensation for received pilots and data
    /* for (int symidx = 0; symidx < d_data_symbols; symidx++)
    {
        unsigned input_offset = (d_preamble_symbols + symidx) * d_scnum;
        unsigned output_offset = (noutsymcnt + symidx) * d_scnum;

        float cpe_phi = d_cpe_offset + symidx * d_cpe_phi;
        const float normfactor = sqrt(float(d_nss * d_scnum) / float(d_fftsize));
        gr_complex cpe_comp = normfactor * std::exp(gr_complex(0, cpe_phi));
        for (int m = 0; m < d_nrx; m++)
        {
            auto *in = (const gr_complex *) input_items[m];
            auto *out = (gr_complex *) output_items[m];
            volk_32fc_s32fc_multiply2_32fc(&out[output_offset], &in[input_offset], &cpe_comp, d_scnum);
        }
    } */
}

void
ltf_chanest_impl::ltf_chan_est_1tx(gr_vector_const_void_star &input_items,
                                   gr_vector_void_star &output_items,
                                   int output_offset)
{
    const int input_offset = d_scnum * (d_preamble_symbols - d_nss);
    const float normfactor =  sqrt(1.0f / float(d_nss));

    for (int m = 0; m < d_nrx; m++)  // for all rx antennas
    {
        auto *in = (const gr_complex *) input_items[m] + input_offset;
        auto *out = (gr_complex *) output_items[m];
        for (int k = 0; k < d_scnum; k++)
        {
            auto hest = normfactor * NORM_LTF_SEQ[k] * in[k];
            d_chan_est[d_nrx * k + m] = hest;  // 1 stream only
            out[output_offset + k] = hest;
        }
    }
}

void
ltf_chanest_impl::ltf_chan_est_2rx(gr_vector_const_void_star &input_items,
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
    for (int k = 0; k < d_scnum; k++)
    {
        Rx(0, 0) = in0[k], Rx(1, 0) = in0[k + d_scnum];  // (s0,r0), (s1,r0)
        Rx(0, 1) = in1[k], Rx(1, 1) = in1[k + d_scnum];  // (s0,r1) (s1,r1)
        // channel estimation H = Pd * Rx, Pd = {1, -1; 1, 1} for 2x2 case
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        Eigen::Map<CMatrix2> H(&d_chan_est[4 * k], 2, 2);
        H = NORM_LTF_SEQ[k] * Pd * Rx;

        // remove cyclic shift
        if (d_remove_cyclic_shift)
        {
            for (int n = 1; n < d_nss; n++) // for all except 1st stream
                for (int m = 0; m < d_nrx; m++) // for all rx
                {
                    int cidx = d_nss * d_nrx * k + d_nrx * n + m; // row major indexing
                    int sidx = n * d_scnum + k;
                    d_chan_est[cidx] *= d_cshift[sidx];
                }
        }

        // copy channel estimation for all streams per receiver antennas to individual output port
        // (s0,r0),(s1,r0) -> ch0,  (s0,r1),(s1,r1) -> ch1
        // using row-major memory layout
        out0[output_offset + 2 * k] = H(0, 0);  // s0,r0
        out0[output_offset + 2 * k + 1] = H(1, 0);  // s1,r0
        out1[output_offset + 2 * k] = H(0, 1);  // s0,r1
        out1[output_offset + 2 * k + 1] = H(1, 1); // s1,r1
    }
}

void
ltf_chanest_impl::ltf_chan_est_nrx(gr_vector_const_void_star &input_items,
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

        // remove cyclic shift
        if (d_remove_cyclic_shift)
        {
            for (int n = 1; n < d_nss; n++) // for all except 1st stream
                for (int m = 0; m < d_nrx; m++) // for all rx
                {
                    int cidx = d_nss * d_nrx * k + d_nrx * n + m; // row major indexing
                    int sidx = n * d_scnum + k;
                    d_chan_est[cidx] *= d_cshift[sidx];
                }
        }

        // copy channel estimation for all streams per receiver antennas to individual output port
        // (s0,r0),(s1,r0) -> ch0,  (s0,r1),(s1,r1) -> ch1
        // using row-major memory layout
        for (int ri = 0; ri < d_nrx; ri++) // for all rx antennas in a set
        {
            auto out = (gr_complex *) output_items[ri];
            for (int m = 0; m < d_nss; m++) // for all streams
                out[output_offset + d_nss * k + m] = H(m, ri);
        }
    }
}

void
ltf_chanest_impl::csi_chan_est_nrx(gr_vector_const_void_star &input_items, int input_offset)
{
    CMatrixX Rx(d_ntx, d_nrx);  // nLTF = nTx for CSI estimation
    // CMatrixX H(d_ntx, d_nrx);   // nSTS = nTx for CSI estimation
    for (int k = 0; k < d_scnum; k++)  // for all subcarriers
    {
        // copy LTF inputs
        for (int m = 0; m < d_nrx; m++)  // for all rx
        {
            auto in = (const gr_complex *) input_items[m] + input_offset;
            for (int n = 0; n < d_ntx; n++)  // for all LTFs
                Rx(n, m) = in[n * d_scnum + k];
        }
        // channel estimation H = Pd * Rx
        // H: nSTS x nRx, Pd: nSTS x nLTF, Rx: nLTF x nRx
        Eigen::Map<CMatrixX> H(&d_chan_csi[d_ntx * d_nrx * k], d_ntx, d_nrx);
        H = NORM_LTF_SEQ[k] * d_Pd * Rx;
    }
    // remove cyclic shift
    for (int k = 0; k < d_scnum; k++)  // for all subcarriers
    {
        for (int n = 1; n < d_ntx; n++) // for all except 1st tx
        {
            for (int m = 0; m < d_nrx; m++) // for all rx
            {
                // int cidx = d_ntx * d_nrx * k + d_ntx * m + n; // column major indexing
                int cidx = d_ntx * d_nrx * k + d_nrx * n + m; // row major indexing
                int sidx = n * d_scnum + k;
                d_chan_csi[cidx] *= d_cshift[sidx];
            }
        }
    }
}

const gr_vector_float ltf_chanest_impl::NORM_LTF_SEQ_64 = {
    1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1,
    1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1,
    -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1};

const gr_vector_float ltf_chanest_impl::NORM_LTF_SEQ_256 = {
    -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1,
    -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1,
    -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1,
    1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1,
    1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1};

} /* namespace gr::ncjt */
