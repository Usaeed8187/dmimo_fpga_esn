/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_MU_CHANEST_H
#define INCLUDED_NCJT_MU_CHANEST_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API mu_chanest : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<mu_chanest> sptr;

/*!
 * \brief Return a shared_ptr to a new instance of ncjt::mu_chanest.
 *
 * To avoid accidental use of raw pointers, ncjt::mu_chanest's
 * constructor is in a private implementation
 * class. ncjt::mu_chanest::make is the public interface for
 * creating new instances.
 */
    static sptr
    make(int fftsize, int ntx, int nrx, int nue, int npreamblesyms,
         int ndatasyms, bool mucpt = false, bool removecs = false, int logfreq = 10, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_MU_CHANEST_H */
