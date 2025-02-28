/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "tx_framing_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

tx_framing::sptr
tx_framing::make(int ntx, int ndatasyms, const char *filename, double fs,
                 int pktspersec, double starttime, int padding, bool autostart, bool debug)
{
    return gnuradio::make_block_sptr<tx_framing_impl>(
        ntx, ndatasyms, filename, fs, pktspersec, starttime, padding, autostart, debug);
}

tx_framing_impl::tx_framing_impl(int ntx, int ndatasyms, const char *filename,
                                 double fs, int pktspersec, double starttime, int padding, bool autostart, bool debug)
    : gr::tagged_stream_block("tx_framing",
                              gr::io_signature::make(ntx, ntx, sizeof(gr_complex)),
                              gr::io_signature::make(ntx, ntx, sizeof(gr_complex)),
                              "packet_len"),
      d_padding_length(padding), d_frame_cnt(0), d_debug(debug)
{
    if (ntx < 1 || ntx > 8)
        throw std::runtime_error("only support 1 to 8 transmitter antennas");
    d_ntx = ntx;

    if (pktspersec <= 0 || pktspersec > 200)
        throw std::runtime_error("invalid sampling packet transmission interval specified");
    d_repeat_interval = 1.0 / (double) pktspersec;

    d_beacon_data = nullptr;
    if ((d_beacon_len = read_beacon_data(filename)) <= 0)
        throw std::runtime_error("failed to read frame data");
    d_beacon_len /= d_ntx;

    d_data_length = ndatasyms * SYM_LEN;
    d_frame_length = d_beacon_len + d_data_length + 2 * d_padding_length;
    if (d_frame_length >= int(0.5 * d_repeat_interval * fs))
        throw std::runtime_error("packet frame too large");

    if (starttime < 0.0 || starttime > 30.0)
        throw std::runtime_error("invalid start time specified");

    double intpart;
    d_time_fracs = std::modf(starttime, &intpart);
    d_time_secs = uint64_t(intpart);
    d_first_burst = true;
    d_txen = autostart;

    d_txtime_offset = starttime;
    while (d_txtime_offset > d_repeat_interval)
        d_txtime_offset -= d_repeat_interval;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    message_port_register_in(pmt::mp("rxtime"));
    set_msg_handler(pmt::mp("rxtime"), [this](const pmt::pmt_t &msg) { process_rxtime_message(msg); });
    message_port_register_in(pmt::mp("txen"));
    set_msg_handler(pmt::mp("txen"), [this](const pmt::pmt_t &msg) { process_txen_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

tx_framing_impl::~tx_framing_impl()
{
    if (d_beacon_data)
        volk_free(d_beacon_data);
}

int
tx_framing_impl::calculate_output_stream_length(
    const gr_vector_int &ninput_items)
{
    return d_frame_length;
}

void
tx_framing_impl::process_rxtime_message(const pmt::pmt_t &msg)
{
    gr::thread::scoped_lock lock(fp_mutex);

    //uint64_t rxtime_secs = pmt::to_uint64(pmt::tuple_ref(msg, 0));
    double rxtime_fracs = pmt::to_double(pmt::tuple_ref(msg, 1));
    //std::cout << "Packet start at " << time_secs << ":" << time_fracs << std::endl;
    while (rxtime_fracs >= d_repeat_interval)  // make it in rage [0, T]
        rxtime_fracs -= d_repeat_interval;

    // timing adjustment using latest rxtime
    double next_txtime =
        (double) (d_time_secs - 1) + rxtime_fracs + d_txtime_offset;
    // schedule time for next transmission
    double schedule_txtime = d_time_fracs + (double) d_time_secs;
    while (next_txtime < (schedule_txtime - 0.05 * d_repeat_interval))
        next_txtime += d_repeat_interval;

    // avoid unnecessary timing adjustment
    if (abs(next_txtime - schedule_txtime) < 1e-5 * d_repeat_interval)
        return;

    // update next transmission time
    double intpart;
    d_time_fracs = std::modf(next_txtime, &intpart);
    d_time_secs = uint64_t(intpart);

    dout << "Adjusted schedule time: " << (double) d_time_secs + d_time_fracs << " seconds" << std::endl;
}

void
tx_framing_impl::process_txen_message(const pmt::pmt_t &msg)
{
    // gr::thread::scoped_lock lock(fp_mutex);
    d_txen = pmt::to_bool(msg);
}

int
tx_framing_impl::work(int noutput_items, gr_vector_int &ninput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    if (noutput_items < d_frame_length)
    {
        std::cout << "Output buffer size: " << noutput_items << std::endl;
        throw std::runtime_error("output buffer size too small");
    }

    // send a null packet on starting or when tx is disabled
    int total_output_items = d_frame_length;
    if (d_first_burst || !d_txen)
    {
        for (int s = 0; s < d_ntx; s++)
        {
            auto out = (gr_complex *) output_items[s];
            memset((void *) &out[0], 0, sizeof(gr_complex) * d_frame_length);

            // Add tags for burst transmission
            add_item_tag(s,
                         nitems_written(s),
                         pmt::string_to_symbol("tx_pkt_len"),
                         pmt::from_long(d_frame_length),
                         _id);

            const pmt::pmt_t tx_time =
                pmt::make_tuple(pmt::from_uint64(d_time_secs), pmt::from_double(d_time_fracs));
            add_item_tag(s,
                         nitems_written(s),
                         pmt::string_to_symbol("tx_time"),
                         tx_time,
                         _id);
        }
        d_first_burst = false;
    }
    else  // send normal packets if transmitter is enabled
    {
        int min_input_items = ninput_items[0];
        for (int s = 1; s < d_ntx; s++)
            min_input_items = std::min(min_input_items, ninput_items[s]);
        if (min_input_items != d_data_length)
        {
            std::cout << "Input data length: " << min_input_items << std::endl;
            throw std::runtime_error("data frame length not correct");
        }

        for (int s = 0; s < d_ntx; s++)
        {
            auto in = (const gr_complex *) input_items[s];
            auto out = (gr_complex *) output_items[s];

            // Add tags for burst transmission
            add_item_tag(s,
                         nitems_written(s),
                         pmt::string_to_symbol("tx_pkt_len"),
                         pmt::from_long(d_frame_length),
                         _id);

            const pmt::pmt_t tx_time =
                pmt::make_tuple(pmt::from_uint64(d_time_secs), pmt::from_double(d_time_fracs));
            add_item_tag(s,
                         nitems_written(s),
                         pmt::string_to_symbol("tx_time"),
                         tx_time,
                         _id);

            // zero-padding and copy beacon symbols
            memset((void *) &out[0], 0, sizeof(gr_complex) * d_padding_length);
            memcpy(&out[d_padding_length], &d_beacon_data[s * d_beacon_len], sizeof(gr_complex) * d_beacon_len);
            // copy data symbols
            memcpy(&out[d_padding_length + d_beacon_len], &in[0], sizeof(gr_complex) * d_data_length);
            // zero-padding after frame
            memset((void *) &out[d_frame_length - d_padding_length], 0, sizeof(gr_complex) * d_padding_length);
        }
    }

    // Update next time stamps
    gr::thread::scoped_lock lock(fp_mutex);
    double intpart; // normalize
    d_frame_cnt += 1;
    d_time_fracs += d_repeat_interval;
    d_time_fracs = std::modf(d_time_fracs, &intpart);
    d_time_secs += uint64_t(intpart);
    dout << "Next schedule time: " << (double) d_time_secs + d_time_fracs << " seconds" << std::endl;

    return total_output_items;
}

int
tx_framing_impl::read_beacon_data(const char *filename)
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
    d_beacon_data = (gr_complex *) malloc(file_size);
    if (data_len != fread(d_beacon_data, sizeof(gr_complex), data_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        free(d_beacon_data);
        return 0;
    }

    return (int) data_len;
}

} /* namespace gr::ncjt */
