/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_STREAM_MAPPER_H
#define INCLUDED_NCJT_STREAM_MAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API stream_mapper : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<stream_mapper> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::stream_mapper.
     *
     * To avoid accidental use of raw pointers, ncjt::stream_mapper's
     * constructor is in a private implementation
     * class. ncjt::stream_mapper::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int rgmode, int nstrm, int phase1_modtype,
         int phase2_modtype, int phase3_modtype, int code_rate,
         bool deterministic_input, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_STREAM_MAPPER_H */
