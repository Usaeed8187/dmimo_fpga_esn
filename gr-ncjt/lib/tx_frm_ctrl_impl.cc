/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "tx_frm_ctrl_impl.h"
#include <gnuradio/io_signature.h>
#include <iomanip>
#include "utils.h"

namespace gr::ncjt
{

tx_frm_ctrl::sptr
tx_frm_ctrl::make(int ntx, int frame_data_len, const char *filename, double samplerate, int pktspersec,
                  double starttime, int padding, bool autostart, int delay, bool debug)
{
    return gnuradio::make_block_sptr<tx_frm_ctrl_impl>(ntx, frame_data_len, filename, samplerate, pktspersec,
                                                       starttime, padding, autostart, delay, debug);
}

tx_frm_ctrl_impl::tx_frm_ctrl_impl(int ntx, int frame_data_len, const char *filename, double samplerate, int pktspersec,
                                   double starttime, int padding, bool autostart, int delay, bool debug)
    : gr::tagged_stream_block(
    "tx_frm_ctrl",
    gr::io_signature::make(ntx, ntx, sizeof(gr_complex)),
    gr::io_signature::make(ntx, ntx, sizeof(gr_complex)),
    "packet_len"),
      d_padding_length(padding), d_cur_frame_cnt(0), d_debug(debug)
{
    if (ntx < 1 || ntx > 8)
        throw std::runtime_error("only support 1 to 8 transmitter antennas");
    d_ntx = ntx;

    if (pktspersec <= 0 || pktspersec > 200)
        throw std::runtime_error("invalid sampling packet transmission interval specified");
    d_pkts_per_sec = pktspersec;
    d_repeat_interval = 1.0 / (double) pktspersec;

    d_samplerate = samplerate;
    d_frame_interval = (uint64_t) round(samplerate / pktspersec);

    if (delay < 0 || delay > 64)
        throw std::runtime_error("invalid delay specified, maximum delay is 64");
    d_delay = delay;

    d_beacon_data = nullptr;
    if ((d_beacon_len = read_beacon_data(filename)) <= 0)
        throw std::runtime_error("failed to read frame data");
    d_beacon_len /= d_ntx;
    dout << "Beacon length: " << d_beacon_len << std::endl;

    if (frame_data_len < 100)
        throw std::runtime_error("invalid frame data length");
    d_data_length = frame_data_len;

    d_frame_length = d_beacon_len + d_data_length + 2 * d_padding_length;
    if (d_frame_length >= int(0.5 * d_repeat_interval * samplerate))
        throw std::runtime_error("packet frame too large");

    if (starttime < 0.0 || starttime > 30.0)
        throw std::runtime_error("invalid start time specified");

    // start time aligned with PPS, its fractional part used as offset for P0/P2/P3
    double intpart;
    d_txtime_offset = std::modf(starttime, &intpart);
    d_txtime_start = uint64_t(intpart);
    // make txtime offset in range (0, repeat_interval)
    while (d_txtime_offset >= d_repeat_interval)
        d_txtime_offset -= d_repeat_interval;
    // adjust for zero padding before preamble
    d_txtime_offset -= double(d_padding_length) / samplerate;

    // timing adjustment from received synchronization beacon
    d_txtime_adjustment = 0;

    // txtime for the initial frame
    d_first_burst = true;
    d_cur_frame_cnt = 0;
    d_prev_frame_start = 0;
    d_prev_frame_cnt = 0;
    d_frame_cnt_adjusted = false;

    // start transmission immediately
    d_txen = autostart;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    message_port_register_in(pmt::mp("rxtime"));
    set_msg_handler(pmt::mp("rxtime"), [this](const pmt::pmt_t &msg) { process_rxtime_message(msg); });
    message_port_register_in(pmt::mp("txen"));
    set_msg_handler(pmt::mp("txen"), [this](const pmt::pmt_t &msg) { process_txen_message(msg); });

    set_tag_propagation_policy(block::TPP_DONT);
}

tx_frm_ctrl_impl::~tx_frm_ctrl_impl()
{
    if (d_beacon_data)
        volk_free(d_beacon_data);
}

int
tx_frm_ctrl_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    return d_frame_length;
}

void
tx_frm_ctrl_impl::process_rxtime_message(const pmt::pmt_t &msg)
{
    gr::thread::scoped_lock lock(fp_mutex);

    uint64_t frame_start_offset = pmt::to_uint64(pmt::tuple_ref(msg, 0));
    double frame_start_time = (double) frame_start_offset / d_samplerate;

    // make frame start position in rage [0, d_frame_interval]
    uint64_t frame_start_pos = frame_start_offset % d_frame_interval;

    // txtime offset adjustment (UHD time offset and P2/P3 offset)
    d_txtime_adjustment = double(frame_start_pos) / double(d_samplerate);

    // save current frame start offset and frame counter
    d_prev_frame_start = frame_start_offset - frame_start_pos;
    d_prev_frame_cnt = d_cur_frame_cnt;

    // save clock offset estimate
    d_clk_offset_est = pmt::to_double(pmt::tuple_ref(msg, 1));

    dout << "Adjust TX timing: " << frame_start_pos << "  Rx timing: "
         << std::fixed << std::setprecision(8) << frame_start_time << std::endl;
}

void
tx_frm_ctrl_impl::process_txen_message(const pmt::pmt_t &msg)
{
    gr::thread::scoped_lock lock(fp_mutex);

    bool txen = pmt::to_bool(msg);
    if (txen != d_txen)
    {
        d_txen = txen;
        dout << "Update TX status: " << d_txen << std::endl;
    }
    if (!d_txen)
        d_frame_cnt_adjusted = false;
}

int
tx_frm_ctrl_impl::work(int noutput_items, gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock lock(fp_mutex);

    if (noutput_items < d_frame_length)
    {
        std::cout << "Output buffer size: " << noutput_items << std::endl;
        throw std::runtime_error("output buffer size too small");
    }

    // adjust frame counter (transmission time) for the first received frame,
    // which will have packet timing different from the current free running frame counter
    if (!d_frame_cnt_adjusted && d_prev_frame_start > 0)
    {
        uint64_t num_tx_frames = d_cur_frame_cnt - d_prev_frame_cnt;
        uint64_t frame_start_adjusted = d_prev_frame_start + (uint64_t) round(d_clk_offset_est * double(num_tx_frames));
        uint64_t min_frame_cnt = (frame_start_adjusted / d_frame_interval) + num_tx_frames;
        if (d_cur_frame_cnt < min_frame_cnt)
        {
            d_cur_frame_cnt = min_frame_cnt;
            d_frame_cnt_adjusted = true;
            std::cout << ">>>>>>>> Adjust TX frame counter to " << min_frame_cnt << std::endl;
        }
    }

    // Update next time stamps
    d_cur_frame_cnt += 1;
    d_time_secs = (d_cur_frame_cnt / d_pkts_per_sec);
    d_time_fracs = double(d_cur_frame_cnt % d_pkts_per_sec) / (double) d_pkts_per_sec;

    double intpart;
    double tx_time_fracs = d_time_fracs + d_txtime_adjustment + d_txtime_offset;
    tx_time_fracs = std::modf(tx_time_fracs, &intpart);
    uint64_t tx_time_secs = d_time_secs + uint64_t(intpart);
    const pmt::pmt_t tx_time =
        pmt::make_tuple(pmt::from_uint64(tx_time_secs), pmt::from_double(tx_time_fracs));

    // send a null packet when starting or tx is disabled
    if (d_first_burst || !d_txen ||  tx_time_secs <= d_txtime_start)
    {
        for (int s = 0; s < d_ntx; s++)
        {
            auto out = (gr_complex *) output_items[s];
            memset((void *) &out[0], 0, sizeof(gr_complex) * d_frame_length);
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

            // zero-padding and copy beacon symbols
            memset((void *) &out[0], 0, sizeof(gr_complex) * (d_padding_length + d_delay));
            memcpy(&out[d_padding_length + d_delay],
                   &d_beacon_data[s * d_beacon_len],
                   sizeof(gr_complex) * d_beacon_len);
            // copy data symbols
            memcpy(&out[d_padding_length + d_beacon_len + d_delay], &in[0], sizeof(gr_complex) * d_data_length);
            // zero-padding after frame
            memset((void *) &out[d_frame_length - d_padding_length + d_delay],
                   0,
                   sizeof(gr_complex) * (d_padding_length - d_delay));
        }
    }

    // Add tags for burst transmission
    for (int ch = 0; ch < d_ntx; ch++)
    {
        add_item_tag(ch,
                     nitems_written(ch),
                     pmt::string_to_symbol("tx_pkt_len"),
                     pmt::from_long(d_frame_length),
                     _id);

        add_item_tag(ch,
                     nitems_written(ch),
                     pmt::string_to_symbol("tx_time"),
                     tx_time,
                     _id);
    }

    return d_frame_length;
}

int
tx_frm_ctrl_impl::read_beacon_data(const char *filename)
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
