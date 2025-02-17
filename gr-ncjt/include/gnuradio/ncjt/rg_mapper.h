/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_RG_MAPPER_H
#define INCLUDED_NCJT_RG_MAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API rg_mapper : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<rg_mapper> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::rg_mapper.
     *
     * To avoid accidental use of raw pointers, ncjt::rg_mapper's
     * constructor is in a private implementation
     * class. ncjt::rg_mapper::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrm, int framelen, int ndatasyms, int npilotsyms, int modtype, bool addcs = true, bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_RG_MAPPER_H */
