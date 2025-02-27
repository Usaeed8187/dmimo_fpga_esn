/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_TX_FRAMING_IMPL_H
#define INCLUDED_NCJT_TX_FRAMING_IMPL_H

#include <gnuradio/ncjt/tx_framing.h>
#include <boost/thread/mutex.hpp>

namespace gr::ncjt {

    class tx_framing_impl : public tx_framing {
    private:
        const int SYM_LEN = 80; // OFDM symbol length (FFT+CP)

        int d_ntx;            // number of transmitter antennas
        int d_frame_length;   // full transmission frame length (beacon, data, and padding)
        int d_beacon_len;     // beacon length in samples
        int d_data_length;    // data symbols length in samples
        int d_padding_length; // zero padding length in samples

        bool d_txen;  // indicate whether transmission is enabled
        bool d_first_burst; // indicate first transmission bust
        double d_repeat_interval;  // repeat transmission interval
        double d_txtime_offset;  // transmission time offset relative to t0
        double d_txtime_adjustment; // current txtime adjustment
        uint64_t d_time_secs;  // integer seconds of next transmission time
        double d_time_fracs;  // fractional seconds of next transmission time
        uint64_t d_frame_cnt;  // total frame counter

        gr_complex *d_beacon_data;  // preamble data

        const bool d_debug;
        pmt::pmt_t _id;

        int
        read_beacon_data(const char *filename);

    protected:
        boost::mutex fp_mutex;

        int
        calculate_output_stream_length(const gr_vector_int &ninput_items);

        void
        process_rxtime_message(const pmt::pmt_t &msg);

        void
        process_txen_message(const pmt::pmt_t &msg);

    public:
        tx_framing_impl(int ntx,
                        int ndatasyms,
                        const char *filename,
                        double fs,
                        int pktspersec,
                        double starttime,
                        int padding,
                        bool autostart,
                        bool debug);

        ~tx_framing_impl();

        int
        work(int noutput_items, gr_vector_int &ninput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
    };

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRAMING_IMPL_H */
