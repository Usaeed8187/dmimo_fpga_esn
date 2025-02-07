/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef INCLUDED_NCJT_API_H
#define INCLUDED_NCJT_API_H

#include <gnuradio/attributes.h>

#ifdef gnuradio_ncjt_EXPORTS
#define NCJT_API __GR_ATTR_EXPORT
#else
#define NCJT_API __GR_ATTR_IMPORT
#endif

#endif /* INCLUDED_NCJT_API_H */
