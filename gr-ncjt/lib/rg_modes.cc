/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include <gnuradio/ncjt/rg_modes.h>

// FFT size
NCJT_API int RG_FFT_SIZE[NUM_RG_MODES] = {
    64, 64, 256, 256, 256, 256, 256, 256
};

// valid subcarriers
NCJT_API int RG_NUM_VALID_SC[NUM_RG_MODES] = {
    56, 56, 242, 242, 140, 140, 140, 140
};

// data subcarriers
NCJT_API int RG_NUM_DATA_SC[NUM_RG_MODES] = {
    52, 52, 234, 234, 132, 132, 132, 132
};

// guard subcarriers on each side
NCJT_API int RG_NUM_GUARD_SC[NUM_RG_MODES] = {
    4, 4, 6, 6, 57, 57, 57, 57
};

// number continuous pilot tones
NCJT_API int RG_NUM_CPT[NUM_RG_MODES] = {
    4, 4, 8, 8, 8, 8, 8, 8
};

// CPT indices
NCJT_API int RG_CPT_INDX[NUM_RG_MODES][MAX_NUM_CPT] = {
    {7, 21, 34, 48, 0, 0, 0, 0},
    {7, 21, 34, 48, 0, 0, 0, 0},
    {6, 32, 74, 100, 141, 167, 209, 235},
    {6, 32, 74, 100, 141, 167, 209, 235},
    {9, 23, 49, 63, 76, 90, 116, 130},
    {9, 23, 49, 63, 76, 90, 116, 130},
    {9, 23, 49, 63, 76, 90, 116, 130},
    {9, 23, 49, 63, 76, 90, 116, 130}
};

// number of OFDM symbols per subframe/slot
NCJT_API int RG_NUM_OFDM_SYM[NUM_RG_MODES] = {
    40, 40, 14, 14, 14, 14, 14, 14
};

// number of DMRS symbols per subframe/slot
NCJT_API int RG_NUM_DMRS_SYM[NUM_RG_MODES] = {
    0, 2, 0, 2, 0, 2, 0, 2
};

// number of CSI-RS symbols per subframe/slot
NCJT_API int RG_NUM_CSIRS_SYM[NUM_RG_MODES] = {
    0, 0, 0, 0, 0, 0, 0, 1
};

// For differential quantization of SNRs
NCJT_API int RB_SIZE[NUM_RG_MODES] = {
    13, 13, 23, 23, 16, 16, 16, 16
};

// bits for the first SNR
NCJT_API int RB_B0[NUM_RG_MODES] = {
    16, 16, 10, 10, 15, 15, 15, 15
};

// bits for each subsequent SNR
NCJT_API int RB_Bd[NUM_RG_MODES] = {
    16, 16, 6, 6, 7, 7, 7, 7
};
