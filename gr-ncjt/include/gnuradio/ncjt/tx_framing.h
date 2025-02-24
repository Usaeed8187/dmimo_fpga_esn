/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_TX_FRAMING_H
#define INCLUDED_NCJT_TX_FRAMING_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt {

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
    class NCJT_API tx_framing : virtual public gr::tagged_stream_block {
    public:
        typedef std::shared_ptr<tx_framing> sptr;

        /*!
         * \brief Return a shared_ptr to a new instance of ncjt::tx_framing.
         *
         * To avoid accidental use of raw pointers, ncjt::tx_framing's
         * constructor is in a private implementation
         * class. ncjt::tx_framing::make is the public interface for
         * creating new instances.
         */
        static sptr
        make(int ntx,
             int ndatasyms,
             const char *filename,
             double fs,
             int pktspersec,
             double starttime,
             int padding,
             bool csirs = false,
             bool debug = false);
    };

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_TX_FRAMING_H */
