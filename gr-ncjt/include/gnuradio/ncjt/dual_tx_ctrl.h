/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_DUAL_TX_CTRL_H
#define INCLUDED_NCJT_DUAL_TX_CTRL_H

#include <gnuradio/block.h>
#include <gnuradio/ncjt/api.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API dual_tx_ctrl : virtual public gr::block
{
public:
    typedef std::shared_ptr <dual_tx_ctrl> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::dual_tx_ctrl.
     *
     * To avoid accidental use of raw pointers, ncjt::dual_tx_ctrl's
     * constructor is in a private implementation
     * class. ncjt::dual_tx_ctrl::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int ntx, double samplerate, int pktspersec, int framelen1,
         const char *beaconfile1, double starttime1, int framelen2,
         const char *beaconfile2, double starttime2, int padding,
         bool autostart, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_DUAL_TX_CTRL_H */
