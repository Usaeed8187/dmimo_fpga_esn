/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_TX_FRM_CTRL_H
#define INCLUDED_NCJT_TX_FRM_CTRL_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API tx_frm_ctrl : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<tx_frm_ctrl> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::tx_frm_ctrl.
     *
     * To avoid accidental use of raw pointers, ncjt::tx_frm_ctrl's
     * constructor is in a private implementation
     * class. ncjt::tx_frm_ctrl::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int ntx, int frame_data_len, const char *filename, double samplerate, int pktspersec,
         double starttime, int padding, bool autostart, int delay, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRM_CTRL_H */
