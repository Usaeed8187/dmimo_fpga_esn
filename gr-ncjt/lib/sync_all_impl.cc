/* -*- c++ -*- */
/*
 * Copyright 2024 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sync_all_impl.h"
#include <gnuradio/io_signature.h>

namespace gr::ncjt
{

sync_all::sptr
sync_all::make(int num_rx, bool debug)
{
    return gnuradio::make_block_sptr<sync_all_impl>(num_rx, debug);
}

sync_all_impl::sync_all_impl(int num_rx, bool debug)
    : gr::block("sync_all",
                gr::io_signature::make(0, 0,
                                       sizeof(float)),
                gr::io_signature::make(0, 0,
                                       sizeof(float))),
      d_debug(debug)
{
    if (num_rx > MAX_NUM_RX || num_rx <= 0)
        throw std::runtime_error("Currently support up to 32 receivers");
    d_num_rx = (unsigned long) num_rx;

    message_port_register_out(pmt::mp("allsync"));

    message_port_register_in(pmt::mp("syncst"));
    set_msg_handler(pmt::mp("syncst"), [this](const pmt::pmt_t &msg) { process_syncst_message(msg); });
}

sync_all_impl::~sync_all_impl()
{
}

void
sync_all_impl::forecast(int noutput_items,
                        gr_vector_int &ninput_items_required)
{
    return;
}

void
sync_all_impl::process_syncst_message(const pmt::pmt_t &msg)
{
    if (!pmt::is_dict(msg) || !pmt::dict_has_key(msg, pmt::mp("rxrdy")))
        return;
    auto rxrdy = pmt::dict_ref(msg, pmt::mp("rxrdy"), pmt::PMT_NIL);
    if (rxrdy != pmt::PMT_NIL) {
        std::string rxid = pmt::symbol_to_string(rxrdy);
        d_rx_state.insert({rxid, true});
        dout << "Receiver ready: " << rxid << std::endl;

        if (d_rx_state.size() == d_num_rx)
        {
            std::cout << "All receiver synchronized" << std::endl;
            send_allsync_message();
        }
    } else {
        dout << "Invalid message format" << std::endl;
    }
}

void
sync_all_impl::send_allsync_message()
{
    message_port_pub(pmt::mp("allsync"), pmt::from_bool(true));
}

int
sync_all_impl::general_work(int noutput_items, gr_vector_int &ninput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
{
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace gr::ncjt */
