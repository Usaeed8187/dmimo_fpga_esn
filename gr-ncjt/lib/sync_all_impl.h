/* -*- c++ -*- */
/*
 * Copyright 2024 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_SYNC_ALL_IMPL_H
#define INCLUDED_NCJT_SYNC_ALL_IMPL_H

#include <gnuradio/ncjt/sync_all.h>
#include "utils.h"

namespace gr::ncjt {

class sync_all_impl : public sync_all {
private:
    const int MAX_NUM_RX = 32; // maximum number of receivers
    unsigned long  d_num_rx;  // total number receivers

    std::map<std::string, bool> d_rx_state;  // receiver state vector

    const bool d_debug;

    void
    process_syncst_message(const pmt::pmt_t &msg);

    void
    send_allsync_message();

public:
  sync_all_impl(int num_rx, bool debug);
  ~sync_all_impl();

  void forecast(int noutput_items, gr_vector_int &ninput_items_required);

  int general_work(int noutput_items, gr_vector_int &ninput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_SYNC_ALL_IMPL_H */
