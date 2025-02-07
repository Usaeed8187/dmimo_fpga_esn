/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_OFDM_MOD_H
#define INCLUDED_NCJT_OFDM_MOD_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API ofdm_mod : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<ofdm_mod> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::ofdm_mod.
     *
     * To avoid accidental use of raw pointers, ncjt::ofdm_mod's
     * constructor is in a private implementation
     * class. ncjt::ofdm_mod::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int fftsize, int cplen, int ntx, float scaling,
         bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_OFDM_MOD_H */
