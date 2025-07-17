/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H
#define GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H

#include <gnuradio/ncjt/api.h>

#define NUM_RG_MODES        8
#define MAX_NUM_CPT         8

extern NCJT_API int RG_FFT_SIZE[NUM_RG_MODES];       // FFT size
extern NCJT_API int RG_NUM_VALID_SC[NUM_RG_MODES];   // valid subcarriers
extern NCJT_API int RG_NUM_DATA_SC[NUM_RG_MODES];    // data subcarriers
extern NCJT_API int RG_NUM_GUARD_SC[NUM_RG_MODES];   // guard subcarriers on each side
extern NCJT_API int RG_NUM_CPT[NUM_RG_MODES];        // number continuous pilot tones
extern NCJT_API int RG_CPT_INDX[NUM_RG_MODES][MAX_NUM_CPT];  // CPT indices

extern NCJT_API int RG_NUM_OFDM_SYM[NUM_RG_MODES];   // number of OFDM symbols per subframe/slot
extern NCJT_API int RG_NUM_DMRS_SYM[NUM_RG_MODES];   // number of DMRS symbols per subframe/slot
extern NCJT_API int RG_NUM_CSIRS_SYM[NUM_RG_MODES];  // number of CSI-RS symbols per subframe/slot

extern NCJT_API int RB_SIZE[NUM_RG_MODES];           // number of subcarriers in a resource block
extern NCJT_API int RB_B0[NUM_RG_MODES];             // bits for the first SNR
extern NCJT_API int RB_Bd[NUM_RG_MODES];             // bits for each subsequent SNR

#endif //GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H
