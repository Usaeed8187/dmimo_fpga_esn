/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_STBC_ENCODE_H
#define INCLUDED_NCJT_STBC_ENCODE_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API stbc_encode : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<stbc_encode> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::stbc_encode.
     *
     * To avoid accidental use of raw pointers, ncjt::stbc_encode's
     * constructor is in a private implementation
     * class. ncjt::stbc_encode::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int ndatasyms, int npilotsyms, int ueidx, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_STBC_ENCODE_H */
