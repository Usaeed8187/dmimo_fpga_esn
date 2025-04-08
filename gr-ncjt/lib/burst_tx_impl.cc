/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "burst_tx_impl.h"
#include <gnuradio/io_signature.h>
#include "utils.h"

namespace gr::ncjt
{

burst_tx::sptr
burst_tx::make(const char *filename, double samplerate,
               int pktspersec, int pktsize, double starttime, bool debug)
{
    return gnuradio::make_block_sptr<burst_tx_impl>(filename, samplerate,
                                                    pktspersec, pktsize, starttime, debug);
}

burst_tx_impl::burst_tx_impl(const char *filename, double samplerate, int pktspersec, int pktsize, double starttime, bool debug)
    : gr::tagged_stream_block("burst_tx",
                              gr::io_signature::make(0, 0, 0),
                              gr::io_signature::make(1, 1, sizeof(gr_complex)),
                              "packet_len"),
      d_first_burst(true), d_debug(debug)
{
    if (pktspersec <= 0 || pktspersec > 100)
        throw std::runtime_error("invalid sampling packet transmission interval specified");
    d_repeat_interval = 1.0 / (double) pktspersec;

    if (pktsize < 0 || pktsize > int(0.5 * d_repeat_interval * samplerate))
        throw std::runtime_error("invalid packet size specified");
    d_pktsize = pktsize;

    d_frame_data = nullptr;
    if ((d_data_len = read_frame_data(filename)) == 0)
        throw std::runtime_error("failed to read frame data");

    if (d_pktsize == 0)
        d_pktsize = d_data_len;

    if (d_pktsize > 0.5 * d_repeat_interval * samplerate)
    {
        std::cout << "Packet size: " << d_pktsize << std::endl;
        throw std::runtime_error("packet size too large");
    }

    if (starttime < 0 || starttime > 10.0)
        throw std::runtime_error("invalid transmission start time");

    d_buf_pos = 0;
    d_time_secs = starttime;
    d_time_fracs = 0.0;

    std::stringstream str;
    str << name() << unique_id();
    _id = pmt::string_to_symbol(str.str());

    set_min_output_buffer(0, 2 * d_pktsize * sizeof(gr_complex));
    set_tag_propagation_policy(block::TPP_DONT);
}

burst_tx_impl::~burst_tx_impl()
{
    if (d_frame_data)
        free(d_frame_data);
}

int
burst_tx_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
{
    return d_pktsize;
}

int
burst_tx_impl::work(int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items)
{
    auto out = (gr_complex *) output_items[0];

    if (noutput_items < d_pktsize)
    {
        std::cout << "Output buffer size: " << noutput_items << std::endl;
        throw std::runtime_error("output buffer size too small");
    }

    int data_remaining = d_data_len - d_buf_pos;
    if (data_remaining == 0) {
        d_buf_pos = 0;
    }
    else if (data_remaining < d_pktsize) {
        throw std::runtime_error("packet fragmentation occurs");
    }

    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("tx_pkt_len"),
                 pmt::from_long(d_pktsize),
                 _id);

    const pmt::pmt_t tx_time = pmt::make_tuple(pmt::from_uint64(d_time_secs), pmt::from_double(d_time_fracs));
    add_item_tag(0,
                 nitems_written(0),
                 pmt::string_to_symbol("tx_time"),
                 tx_time,
                 _id);

    memcpy(&out[0], &d_frame_data[d_buf_pos], sizeof(gr_complex) * d_pktsize);

    // Update next time stamps
    if (d_first_burst)
    {
        d_first_burst = false;
        d_time_secs = 2;
    }
    d_buf_pos += d_pktsize;
    d_time_fracs += d_repeat_interval;
    double intpart; // normalize
    d_time_fracs = std::modf(d_time_fracs, &intpart);
    d_time_secs += uint64_t(intpart);

    return d_pktsize;
}

uint64_t
burst_tx_impl::read_frame_data(const char *filename)
{
    FILE *d_fp;
    struct GR_STAT st;

    if ((d_fp = fopen(filename, "rb")) == nullptr)
        return 0;
    if (GR_FSTAT(GR_FILENO(d_fp), &st))
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_END);
    uint64_t file_size = GR_FTELL(d_fp);
    uint64_t packet_len = file_size / sizeof(gr_complex);
    if (packet_len == 0)
        return 0;

    GR_FSEEK(d_fp, 0, SEEK_SET);
    d_frame_data = (gr_complex *) malloc(file_size);
    if (d_frame_data == nullptr)
        throw std::runtime_error("failed to allocate packet buffer");

    if (packet_len != fread(d_frame_data, sizeof(gr_complex), packet_len, d_fp))
    {
        dout << "failed to read file content" << std::endl;
        free(d_frame_data);
        return 0;
    }

    return packet_len;
}

} /* namespace gr::ncjt */
