/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#ifndef GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H
#define GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H

#define NUM_RG_MODES        8
#define MAX_NUM_CPT         8

extern int RG_FFT_SIZE[NUM_RG_MODES];       // FFT size
extern int RG_NUM_VALID_SC[NUM_RG_MODES];   // valid subcarriers
extern int RG_NUM_DATA_SC[NUM_RG_MODES];    // data subcarriers
extern int RG_NUM_GUARD_SC[NUM_RG_MODES];   // guard subcarriers on each side
extern int RG_NUM_CPT[NUM_RG_MODES];        // number continuous pilot tones
extern int RG_CPT_INDX[NUM_RG_MODES][MAX_NUM_CPT];  // CPT indices

extern int RG_NUM_OFDM_SYM[NUM_RG_MODES];   // number of OFDM symbols per subframe/slot
extern int RG_NUM_DMRS_SYM[NUM_RG_MODES];   // number of DMRS symbols per subframe/slot
extern int RG_NUM_CSIRS_SYM[NUM_RG_MODES];  // number of CSI-RS symbols per subframe/slot

#endif //GR_NCJT_INCLUDE_GNURADIO_NCJT_RG_MODES_H
