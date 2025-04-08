/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_LLR_COMBINE_H
#define INCLUDED_NCJT_LLR_COMBINE_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API llr_combine : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<llr_combine> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::llr_combine.
     *
     * To avoid accidental use of raw pointers, ncjt::llr_combine's
     * constructor is in a private implementation
     * class. ncjt::llr_combine::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, int modtype1, int modtype2, int blocksize,
         bool llrsum = false, bool majority = false, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_LLR_COMBINE_H */
