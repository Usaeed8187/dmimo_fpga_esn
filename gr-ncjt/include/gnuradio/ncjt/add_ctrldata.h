/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_ADD_CTRLDATA_H
#define INCLUDED_NCJT_ADD_CTRLDATA_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API add_ctrldata : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<add_ctrldata> sptr;

/*!
 * \brief Return a shared_ptr to a new instance of ncjt::add_ctrldata.
 *
 * To avoid accidental use of raw pointers, ncjt::add_ctrldata's
 * constructor is in a private implementation
 * class. ncjt::add_ctrldata::make is the public interface for
 * creating new instances.
 */
    static sptr
    make(int nstrm, int ctrl_datalen, bool add_txseq, bool add_rxseq, bool add_llr, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_ADD_CTRLDATA_H */
