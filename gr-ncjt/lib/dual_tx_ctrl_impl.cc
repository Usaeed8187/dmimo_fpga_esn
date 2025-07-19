/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "dual_tx_ctrl_impl.h"
#include <gnuradio/io_signature.h>
#include <iomanip>
#include "utils.h"

namespace gr::ncjt
{

dual_tx_ctrl::sptr
dual_tx_ctrl::make(int ntx, double samplerate, int pktspersec, int framelen1,
                   const char *beaconfile1, double starttime1, int framelen2,
                   const char *beaconfile2, double starttime2, int padding,
                   bool autostart, bool debug)
{
    return gnuradio::make_block_sptr<dual_tx_ctrl_impl>(
        ntx, samplerate, pktspersec, framelen1, beaconfile1, starttime1,
        framelen2, beaconfile2, starttime2, padding, autostart, debug);
}

dual_tx_ctrl_impl::dual_tx_ctrl_impl(int ntx, double samplerate, int pktspersec,
                                     int framelen1, const char *beaconfile1,
                                     double starttime1, int framelen2,
                                     const char *beaconfile2, double starttime2,
                                     int padding, bool autostart, bool debug)
    : gr::block("dual_tx_ctrl",
                gr::io_signature::make(2 * ntx, 2 * ntx, sizeof(gr_complex)),
                gr::io_signature::make(ntx, ntx, sizeof(gr_complex))),
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

    if ((d_beacon_data1 = read_beacon_data(beaconfile1, &d_beacon_len1)) == nullptr)
        throw std::runtime_error("failed to read first frame data");
    d_beacon_len1 /= d_ntx;
    dout << "Beacon length: " << d_beacon_len1 << std::endl;

    if (framelen1 < 100)
        throw std::runtime_error("invalid frame data length");
    d_data_length1 = framelen1;

    d_frame_length1 = d_beacon_len1 + d_data_length1 + 2 * d_padding_length;
    if (d_frame_length1 >= int(0.5 * d_repeat_interval * samplerate))
        throw std::runtime_error("packet frame too large");

    if ((d_beacon_data2 = read_beacon_data(beaconfile2, &d_beacon_len2)) == nullptr)
        throw std::runtime_error("failed to read second frame data");
    d_beacon_len2 /= d_ntx;
    dout << "Beacon length: " << d_beacon_len2 << std::endl;

    if (framelen2 < 100)
        throw std::runtime_error("invalid frame data length");
    d_data_length2 = framelen2;

    d_frame_length2 = d_beacon_len2 + d_data_length2 + 2 * d_padding_length;
    if (d_frame_length2 >= int(0.5 * d_repeat_interval * samplerate))
        throw std::runtime_error("packet frame too large");

    if (starttime1 < 0.0 || starttime1 > 30.0 || starttime2 < 0.0 || starttime2 > 30.0)
        throw std::runtime_error("invalid start time specified");

    // start time aligned with PPS, its fractional part used as offset for P0/P2/P3
    double intpart1;
    d_txtime_offset1 = std::modf(starttime1, &intpart1);
    // make txtime offset in range (0, repeat_interval)
    while (d_txtime_offset1 >= d_repeat_interval)
        d_txtime_offset1 -= d_repeat_interval;
    // adjust for zero padding before preamble
    d_txtime_offset1 -= double(d_padding_length) / samplerate;

    // start time aligned with PPS, its fractional part used as offset for P0/P2/P3
    double intpart2;
    d_txtime_offset2 = std::modf(starttime2, &intpart2);
    // make txtime offset in range (0, repeat_interval)
    while (d_txtime_offset2 >= d_repeat_interval)
        d_txtime_offset2 -= d_repeat_interval;
    // adjust for zero padding before preamble
    d_txtime_offset2 -= double(d_padding_length) / samplerate;

    if (d_txtime_offset2 < d_txtime_offset1 + 0.01 * d_repeat_interval)
        throw std::runtime_error("start time for Tx 1 must be substantially less than Tx 2");

    d_txtime_start = (uint64_t) std::max(intpart1, intpart2);

    // timing adjustment from received synchronization beacon
    d_txtime_adjustment = 0;

    // txtime for the initial frame
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

dual_tx_ctrl_impl::~dual_tx_ctrl_impl()
{
    if (d_beacon_data1)
        volk_free(d_beacon_data1);
    if (d_beacon_data2)
        volk_free(d_beacon_data2);
}

void
dual_tx_ctrl_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    // input buffers can be empty
    for (int k = 0; k < 2 * d_ntx; k++)
        ninput_items_required[k] = 0;
}

void
dual_tx_ctrl_impl::process_rxtime_message(const pmt::pmt_t &msg)
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
dual_tx_ctrl_impl::process_txen_message(const pmt::pmt_t &msg)
{
    gr::thread::scoped_lock lock(fp_mutex);

    bool txen = (pmt::to_long(msg) > 0);
    if (txen != d_txen)
    {
        d_txen = txen;
        dout << "Update TX status: " << d_txen << std::endl;
    }
    if (!d_txen)
        d_frame_cnt_adjusted = false;
}

int
dual_tx_ctrl_impl::general_work(int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock lock(fp_mutex);

    // transmit when buffers are not empty, transmission rate limited by the USRP hardware
    if (noutput_items < (d_frame_length1 + d_frame_length2))
        return 0;

    int min_input_items = ninput_items[0];
    for (int ch = 1; ch < d_ntx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    bool tx1_ready = (min_input_items >= d_data_length1);

    min_input_items = ninput_items[d_ntx];
    for (int ch = d_ntx + 1; ch < 2 * d_ntx; ch++)
        min_input_items = std::min(min_input_items, ninput_items[ch]);
    bool tx2_ready = (min_input_items >= d_data_length2);

    /* if (!d_frame_cnt_adjusted && d_prev_frame_start > 0)
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
    } */

    // Update next time stamps
    d_cur_frame_cnt += 1;
    d_time_secs = (d_cur_frame_cnt / d_pkts_per_sec);
    d_time_fracs = double(d_cur_frame_cnt % d_pkts_per_sec) / (double) d_pkts_per_sec;

    // First transmit channel

    double intpart;
    double tx_time_fracs = d_time_fracs + d_txtime_adjustment + d_txtime_offset1;
    tx_time_fracs = std::modf(tx_time_fracs, &intpart);
    uint64_t tx_time_secs = d_time_secs + uint64_t(intpart);
    const pmt::pmt_t tx_time_1 =
        pmt::make_tuple(pmt::from_uint64(tx_time_secs), pmt::from_double(tx_time_fracs));

    if (!(tx1_ready && tx2_ready) && d_txen && tx_time_secs <= d_txtime_start)
        return 0;

    // Send a null packet when starting or tx is disabled
    if (!tx1_ready || !d_txen || tx_time_secs <= d_txtime_start)
    {
        for (int s = 0; s < d_ntx; s++)
        {
            auto out = (gr_complex *) output_items[s];
            memset((void *) &out[0], 0, sizeof(gr_complex) * d_frame_length1);
        }
    }
    else  // Send normal packets if transmitter is enabled
    {
        // check stream tags
        std::vector<gr::tag_t> d_tags;
        get_tags_in_window(d_tags, 0, 0, 1, pmt::string_to_symbol("packet_len"));
        if (d_tags.empty())
            throw std::runtime_error("packet length tag not found");

        for (int s = 0; s < d_ntx; s++)
        {
            auto in = (const gr_complex *) input_items[s];
            auto out = (gr_complex *) output_items[s];

            // zero-padding and copy beacon symbols
            memset((void *) &out[0], 0, sizeof(gr_complex) * d_padding_length);
            memcpy(&out[d_padding_length], &d_beacon_data1[s * d_beacon_len1],
                   sizeof(gr_complex) * d_beacon_len1);
            // copy data symbols
            memcpy(&out[d_padding_length + d_beacon_len1], &in[0],
                   sizeof(gr_complex) * d_data_length1);
            // zero-padding after frame
            memset((void *) &out[d_frame_length1 - d_padding_length], 0,
                   sizeof(gr_complex) * d_padding_length);
        }
    }

    // Add tags for burst transmission
    uint64_t offset = nitems_written(0);
    add_packet_tags(offset, d_frame_length1, tx_time_1);

    // Second transmit channel

    tx_time_fracs = d_time_fracs + d_txtime_adjustment + d_txtime_offset2;
    tx_time_fracs = std::modf(tx_time_fracs, &intpart);
    tx_time_secs = d_time_secs + uint64_t(intpart);
    const pmt::pmt_t tx_time_2 =
        pmt::make_tuple(pmt::from_uint64(tx_time_secs), pmt::from_double(tx_time_fracs));

    // Send a null packet when starting or tx is disabled
    int output_offset = d_frame_length1;
    if (!tx2_ready || !d_txen || tx_time_secs <= d_txtime_start)
    {
        for (int s = 0; s < d_ntx; s++)
        {
            auto out = (gr_complex *) output_items[s];
            memset((void *) &out[output_offset], 0, sizeof(gr_complex) * d_frame_length2);
        }
    }
    else  // Send normal packets if transmitter is enabled
    {
        // check stream tags
        std::vector<gr::tag_t> d_tags;
        get_tags_in_window(d_tags, d_ntx, 0, 1, pmt::string_to_symbol("packet_len"));
        if (d_tags.empty())
            throw std::runtime_error("packet length tag not found");

        for (int s = 0; s < d_ntx; s++)
        {
            auto in = (const gr_complex *) input_items[d_ntx + s];
            auto out = (gr_complex *) output_items[s];

            // zero-padding and copy beacon symbols
            memset((void *) &out[output_offset], 0, sizeof(gr_complex) * d_padding_length);
            memcpy(&out[output_offset + d_padding_length], &d_beacon_data2[s * d_beacon_len2],
                   sizeof(gr_complex) * d_beacon_len2);
            // copy data symbols
            memcpy(&out[output_offset + d_padding_length + d_beacon_len2], &in[0],
                   sizeof(gr_complex) * d_data_length2);
            // zero-padding after frame
            memset((void *) &out[output_offset + d_frame_length2 - d_padding_length], 0,
                   sizeof(gr_complex) * d_padding_length);
        }
    }

    // Add tags for burst transmission
    offset = nitems_written(0) + output_offset;
    add_packet_tags(offset, d_frame_length2, tx_time_2);

    // Consume input buffers
    if (tx1_ready)
    {
        for (int ch = 0; ch < d_ntx; ch++)
            consume(ch, d_data_length1);
    }
    if (tx2_ready)
    {
        for (int ch = 0; ch < d_ntx; ch++)
            consume(d_ntx + ch, d_data_length2);
    }

    return (d_frame_length1 + d_frame_length2);
}

void
dual_tx_ctrl_impl::add_packet_tags(uint64_t offset, int pkt_len, const pmt::pmt_t &tx_time)
{
    // Add tags for burst transmission
    for (int ch = 0; ch < d_ntx; ch++)
    {
        add_item_tag(ch,
                     offset,
                     pmt::string_to_symbol("tx_pkt_len"),
                     pmt::from_long(pkt_len),
                     _id);

        add_item_tag(ch,
                     offset,
                     pmt::string_to_symbol("tx_time"),
                     tx_time,
                     _id);
    }
}

gr_complex *
dual_tx_ctrl_impl::read_beacon_data(const char *filename, int *beacon_len)
{
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return nullptr;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return nullptr;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t data_len = file_size / sizeof(gr_complex);
    if (data_len == 0)
        return nullptr;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    auto beacon_data = (gr_complex *) malloc(file_size);
    if (data_len != fread(beacon_data, sizeof(gr_complex), data_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        free(beacon_data);
        return nullptr;
    }
    (*beacon_len) = (int) data_len;

    return beacon_data;
}

} /* namespace gr::ncjt */
