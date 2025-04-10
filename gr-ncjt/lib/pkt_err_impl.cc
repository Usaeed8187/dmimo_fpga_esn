/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "pkt_err_impl.h"
#include <iomanip>
#include <gnuradio/io_signature.h>
#include "utils.h"

using namespace std;

namespace gr::ncjt
{

pkt_err::sptr
pkt_err::make(int nstrms, const char *filename, bool shortterm, int avgwindow,
              int logfreq, bool logpkterr, bool debug)
{
    return gnuradio::make_block_sptr<pkt_err_impl>(nstrms, filename, shortterm, avgwindow, logfreq, logpkterr, debug);
}

pkt_err_impl::pkt_err_impl(int nstrms, const char *filename, bool shortterm, int avgwindow,
                           int logfreq, bool logpkterr, bool debug)
    : gr::tagged_stream_block("pkt_err",
                              gr::io_signature::make(nstrms, nstrms, sizeof(char)),
                              gr::io_signature::make2(1, 2, sizeof(float), nstrms * sizeof(int)),
                              "packet_len"),
      d_shortterm(shortterm), d_log_freq(logfreq), d_logpkterr(logpkterr), d_debug(debug)
{
    if (nstrms < 1 || nstrms > 8)
        throw std::runtime_error("only sport 1 to 8 data channels");
    d_num_strms = nstrms;

    if (avgwindow < 1)
        throw std::runtime_error("invalid average window length");
    d_averaging_window = avgwindow;

    d_pkt_data = nullptr;
    if ((d_pkt_len = read_packet_data(filename)) == 0)
    {
        throw std::runtime_error("failed to read packet data");
    }
    d_pkt_len /= d_num_strms;

    d_total_pkts = 0;
    d_cur_pos = 0;
    d_cur_bit_errs_sum = 0;
    for (int k = 0; k < 8; k++)
    {
        d_cur_bit_errs[k] = 0;
        d_total_bit_errs[k] = 0;
        d_total_pkt_errs[k] = 0;
    }

    d_ber_history = std::deque<double>(d_averaging_window, 0.0);
    d_ber_history_sum = 0.0;

    message_port_register_in(pmt::mp("reset"));
    set_msg_handler(pmt::mp("reset"), [this](const pmt::pmt_t &msg) { process_reset_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

pkt_err_impl::~pkt_err_impl()
{
    if (d_pkt_data != nullptr)
        volk_free(d_pkt_data);
}

int
pkt_err_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_num_strms; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    return d_num_strms * (min_input_items / d_pkt_len);
}

void
pkt_err_impl::process_reset_message(const pmt::pmt_t &msg)
{
    d_total_pkts = 0;
    d_cur_pos = 0;
    d_cur_bit_errs_sum = 0;
    for (int k = 0; k < 8; k++)
    {
        d_cur_bit_errs[k] = 0;
        d_total_bit_errs[k] = 0;
        d_total_pkt_errs[k] = 0;
    }

    if (d_shortterm)
    {
        d_ber_history.clear();
        d_ber_history = std::deque<double>(d_averaging_window, 0.0);
        d_ber_history_sum = 0.0;
    }
}

int
pkt_err_impl::work(int noutput_items, gr_vector_int &ninput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items)
{
    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_num_strms; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);

    if (min_input_items == 0 || d_pkt_data == nullptr)
        return 0;

    bool has_pkterr_out = (d_logpkterr && output_items.size() > 1);
    int *pkterrout = (has_pkterr_out) ? (int *) output_items[1] : nullptr;

    uint64_t output_cnt = 0;
    uint64_t berout_cnt = 0;
    uint64_t bit_pos = d_cur_pos;
    for (int k = 0; k < min_input_items; k++)
    {
        for (int ch = 0; ch < d_num_strms; ch++)
        {
            const char *in = (const char *) input_items[ch];
            if (in[k] - d_pkt_data[ch * d_pkt_len + bit_pos] != 0)
            {
                d_total_bit_errs[ch]++;
                d_cur_bit_errs[ch]++;
            }
        }
        bit_pos++;

        if (bit_pos == d_pkt_len)
        {
            bit_pos = 0;
            int64_t current_packet_bit_errors = 0;
            int64_t total_bit_errors = 0;
            for (int ch = 0; ch < d_num_strms; ch++)
            {
                if (has_pkterr_out)   // packet BER output
                    pkterrout[output_cnt * d_num_strms + ch] = d_cur_bit_errs[ch];
                // update current error statistics
                if (d_cur_bit_errs[ch] > 0)
                    d_total_pkt_errs[ch]++;
                d_cur_bit_errs_sum += d_cur_bit_errs[ch];
                current_packet_bit_errors += d_cur_bit_errs[ch];
                d_cur_bit_errs[ch] = 0;
                // culminating stats
                total_bit_errors += d_total_bit_errs[ch];
            }
            output_cnt++;
            d_total_pkts++;

            // culminating stats
            double bit_err_rate = (double) total_bit_errors / (double) (d_num_strms * d_total_pkts * d_pkt_len);
            // short-term stats
            double short_term_bit_err_rate;
            if (d_shortterm)
            {
                double current_packet_ber = (double) current_packet_bit_errors / (double) (d_num_strms * d_pkt_len);
                d_ber_history_sum -= d_ber_history.front();
                d_ber_history.pop_front();
                d_ber_history.push_back(current_packet_ber);
                d_ber_history_sum += current_packet_ber;
                short_term_bit_err_rate = d_ber_history_sum / (double) d_averaging_window;
            }

            // BER monitor output
            auto berout = (float *) output_items[0];
            if (d_shortterm)
                berout[berout_cnt] = (float) short_term_bit_err_rate;
            else
                berout[berout_cnt] = (float) bit_err_rate;
            berout_cnt++;

            // debug output
            if (d_total_pkts % d_log_freq == 0)
            {
                dout << ">>>>>>>> Total Packets: " << d_total_pkts << endl;
                uint64_t total_pkt_errs_sum = 0, total_bit_errs_sum = 0;
                for (int ch = 0; ch < d_num_strms; ch++)
                {
                    total_pkt_errs_sum += d_total_pkt_errs[ch];
                    total_bit_errs_sum += d_total_bit_errs[ch];
                    double pkt_err_rate_ch = d_total_pkt_errs[ch] / (double) d_total_pkts;
                    double bit_err_rate_ch = d_total_bit_errs[ch] / (double) (d_total_pkts * d_pkt_len);
                    dout << "    PER[" << ch << "]: " << left << setw(22) << pkt_err_rate_ch
                         << "\tBER[" << ch << "]: " << bit_err_rate_ch << endl;
                }
                dout << "    PERavg: " << left << setw(21)
                     << double(total_pkt_errs_sum) / (d_num_strms * d_total_pkts)
                     << "\tBERavg: " << double(total_bit_errs_sum) / (d_num_strms * d_total_pkts * d_pkt_len) << endl;
                dout << ">>>>>>>> Current BER: "
                     << (double) d_cur_bit_errs_sum / (double) (d_log_freq * d_num_strms * d_pkt_len) << endl;
                d_cur_bit_errs_sum = 0;
            }
        }
    }
    d_cur_pos = bit_pos;

    produce(0, berout_cnt);
    if (has_pkterr_out)
        produce(1, output_cnt);

    return WORK_CALLED_PRODUCE;
}

uint64_t
pkt_err_impl::read_packet_data(const char *filename)
{
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return 0;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t packet_len = file_size / sizeof(char);
    if (packet_len == 0)
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    d_pkt_data = (char *) malloc(file_size);
    if (packet_len != fread(d_pkt_data, sizeof(char), packet_len, d_fp))
    {
        dout << "failed to read file content" << endl;
        free(d_pkt_data);
        return 0;
    }

    return packet_len;
}

} /* namespace gr::ncjt */
