/* -*- c++ -*- */
/*
 * Copyright 2025 gr-ncjt author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_NCJT_UL_PRECODING_H
#define INCLUDED_NCJT_UL_PRECODING_H

#include <gnuradio/ncjt/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr::ncjt
{

/*!
 * \brief <+description of block+>
 * \ingroup ncjt
 *
 */
class NCJT_API ul_precoding
: virtual public gr::tagged_stream_block
{
public:
typedef std::shared_ptr <ul_precoding> sptr;

/*!
 * \brief Return a shared_ptr to a new instance of ncjt::ul_precoding.
 *
 * To avoid accidental use of raw pointers, ncjt::ul_precoding's
 * constructor is in a private implementation
 * class. ncjt::ul_precoding::make is the public interface for
 * creating new instances.
 */
static sptr
make(int nstrms, int numsyms, bool debug);
};

} // namespace gr::ncjt

#endif /* INCLUDED_NCJT_UL_PRECODING_H */
