/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_QAM_DEMAPPER_H
#define INCLUDED_NCJT_QAM_DEMAPPER_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API qam_demapper : virtual public gr::tagged_stream_block
{
public:
    typedef std::shared_ptr<qam_demapper> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ncjt::qam_demapper.
     *
     * To avoid accidental use of raw pointers, ncjt::qam_demapper's
     * constructor is in a private implementation
     * class. ncjt::qam_demapper::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(int nstrms, int modtype, bool usecsi = false,
         bool debug = false);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_QAM_DEMAPPER_H */
