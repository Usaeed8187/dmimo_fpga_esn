/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "qam_constellation.h"

namespace gr::ncjt
{

const
std::vector<gr_complex> CONST_QPSK = {
    gr_complex(-0.707106781186547, -0.707106781186547), gr_complex(-0.707106781186547, 0.707106781186547),
    gr_complex(0.707106781186547, -0.707106781186547), gr_complex(0.707106781186547, 0.707106781186547)
};

#define Q16V3   0.948683298050514
#define Q16V1   0.316227766016838
const
std::vector<gr_complex> CONST_16QAM = {
    gr_complex(-Q16V3, -Q16V3), gr_complex(-Q16V3, -Q16V1),
    gr_complex(-Q16V3, Q16V3), gr_complex(-Q16V3, Q16V1),
    gr_complex(-Q16V1, -Q16V3), gr_complex(-Q16V1, -Q16V1),
    gr_complex(-Q16V1, Q16V3), gr_complex(-Q16V1, Q16V1),
    gr_complex(Q16V3, -Q16V3), gr_complex(Q16V3, -Q16V1),
    gr_complex(Q16V3, Q16V3), gr_complex(Q16V3, Q16V1),
    gr_complex(Q16V1, -Q16V3), gr_complex(Q16V1, -Q16V1),
    gr_complex(Q16V1, Q16V3), gr_complex(Q16V1, Q16V1)
};

#define Q64V7   1.080123449734643
#define Q64V5   0.771516749810460
#define Q64V3   0.462910049886276
#define Q64V1   0.154303349962092

const
std::vector<gr_complex> CONST_64QAM = {
    gr_complex(-Q64V7, -Q64V7), gr_complex(-Q64V7, -Q64V5), gr_complex(-Q64V7, -Q64V1), gr_complex(-Q64V7, -Q64V3),
    gr_complex(-Q64V7, Q64V7), gr_complex(-Q64V7, Q64V5), gr_complex(-Q64V7, Q64V1), gr_complex(-Q64V7, Q64V3),
    gr_complex(-Q64V5, -Q64V7), gr_complex(-Q64V5, -Q64V5), gr_complex(-Q64V5, -Q64V1), gr_complex(-Q64V5, -Q64V3),
    gr_complex(-Q64V5, Q64V7), gr_complex(-Q64V5, Q64V5), gr_complex(-Q64V5, Q64V1), gr_complex(-Q64V5, Q64V3),
    gr_complex(-Q64V1, -Q64V7), gr_complex(-Q64V1, -Q64V5), gr_complex(-Q64V1, -Q64V1), gr_complex(-Q64V1, -Q64V3),
    gr_complex(-Q64V1, Q64V7), gr_complex(-Q64V1, Q64V5), gr_complex(-Q64V1, Q64V1), gr_complex(-Q64V1, Q64V3),
    gr_complex(-Q64V3, -Q64V7), gr_complex(-Q64V3, -Q64V5), gr_complex(-Q64V3, -Q64V1), gr_complex(-Q64V3, -Q64V3),
    gr_complex(-Q64V3, Q64V7), gr_complex(-Q64V3, Q64V5), gr_complex(-Q64V3, Q64V1), gr_complex(-Q64V3, Q64V3),
    gr_complex(Q64V7, -Q64V7), gr_complex(Q64V7, -Q64V5), gr_complex(Q64V7, -Q64V1), gr_complex(Q64V7, -Q64V3),
    gr_complex(Q64V7, Q64V7), gr_complex(Q64V7, Q64V5), gr_complex(Q64V7, Q64V1), gr_complex(Q64V7, Q64V3),
    gr_complex(Q64V5, -Q64V7), gr_complex(Q64V5, -Q64V5), gr_complex(Q64V5, -Q64V1), gr_complex(Q64V5, -Q64V3),
    gr_complex(Q64V5, Q64V7), gr_complex(Q64V5, Q64V5), gr_complex(Q64V5, Q64V1), gr_complex(Q64V5, Q64V3),
    gr_complex(Q64V1, -Q64V7), gr_complex(Q64V1, -Q64V5), gr_complex(Q64V1, -Q64V1), gr_complex(Q64V1, -Q64V3),
    gr_complex(Q64V1, Q64V7), gr_complex(Q64V1, Q64V5), gr_complex(Q64V1, Q64V1), gr_complex(Q64V1, Q64V3),
    gr_complex(Q64V3, -Q64V7), gr_complex(Q64V3, -Q64V5), gr_complex(Q64V3, -Q64V1), gr_complex(Q64V3, -Q64V3),
    gr_complex(Q64V3, Q64V7), gr_complex(Q64V3, Q64V5), gr_complex(Q64V3, Q64V1), gr_complex(Q64V3, Q64V3)
};

#define Q256V15  1.150447483271056
#define Q256V13  0.997054485501581
#define Q256V11  0.843661487732107
#define Q256V9   0.690268489962633
#define Q256V7   0.536875492193159
#define Q256V5   0.383482494423685
#define Q256V3   0.230089496654211
#define Q256V1   0.076696498884737

const
std::vector<gr_complex> CONST_256QAM = {
    gr_complex(-Q256V15, -Q256V5), gr_complex(-Q256V15, -Q256V7), gr_complex(-Q256V15, -Q256V3),
    gr_complex(-Q256V15, -Q256V1),
    gr_complex(-Q256V15, -Q256V11), gr_complex(-Q256V15, -Q256V9), gr_complex(-Q256V15, -Q256V13),
    gr_complex(-Q256V15, -Q256V15),
    gr_complex(-Q256V15, Q256V5), gr_complex(-Q256V15, Q256V7), gr_complex(-Q256V15, Q256V3),
    gr_complex(-Q256V15, Q256V1),
    gr_complex(-Q256V15, Q256V11), gr_complex(-Q256V15, Q256V9), gr_complex(-Q256V15, Q256V13),
    gr_complex(-Q256V15, Q256V15),

    gr_complex(-Q256V13, -Q256V5), gr_complex(-Q256V13, -Q256V7), gr_complex(-Q256V13, -Q256V3),
    gr_complex(-Q256V13, -Q256V1),
    gr_complex(-Q256V13, -Q256V11), gr_complex(-Q256V13, -Q256V9), gr_complex(-Q256V13, -Q256V13),
    gr_complex(-Q256V13, -Q256V15),
    gr_complex(-Q256V13, Q256V5), gr_complex(-Q256V13, Q256V7), gr_complex(-Q256V13, Q256V3),
    gr_complex(-Q256V13, Q256V1),
    gr_complex(-Q256V13, Q256V11), gr_complex(-Q256V13, Q256V9), gr_complex(-Q256V13, Q256V13),
    gr_complex(-Q256V13, Q256V15),

    gr_complex(-Q256V9, -Q256V5), gr_complex(-Q256V9, -Q256V7), gr_complex(-Q256V9, -Q256V3),
    gr_complex(-Q256V9, -Q256V1),
    gr_complex(-Q256V9, -Q256V11), gr_complex(-Q256V9, -Q256V9), gr_complex(-Q256V9, -Q256V13),
    gr_complex(-Q256V9, -Q256V15),
    gr_complex(-Q256V9, Q256V5), gr_complex(-Q256V9, Q256V7), gr_complex(-Q256V9, Q256V3), gr_complex(-Q256V9, Q256V1),
    gr_complex(-Q256V9, Q256V11), gr_complex(-Q256V9, Q256V9), gr_complex(-Q256V9, Q256V13),
    gr_complex(-Q256V9, Q256V15),

    gr_complex(-Q256V11, -Q256V5), gr_complex(-Q256V11, -Q256V7), gr_complex(-Q256V11, -Q256V3),
    gr_complex(-Q256V11, -Q256V1),
    gr_complex(-Q256V11, -Q256V11), gr_complex(-Q256V11, -Q256V9), gr_complex(-Q256V11, -Q256V13),
    gr_complex(-Q256V11, -Q256V15),
    gr_complex(-Q256V11, Q256V5), gr_complex(-Q256V11, Q256V7), gr_complex(-Q256V11, Q256V3),
    gr_complex(-Q256V11, Q256V1),
    gr_complex(-Q256V11, Q256V11), gr_complex(-Q256V11, Q256V9), gr_complex(-Q256V11, Q256V13),
    gr_complex(-Q256V11, Q256V15),

    gr_complex(-Q256V1, -Q256V5), gr_complex(-Q256V1, -Q256V7), gr_complex(-Q256V1, -Q256V3),
    gr_complex(-Q256V1, -Q256V1),
    gr_complex(-Q256V1, -Q256V11), gr_complex(-Q256V1, -Q256V9), gr_complex(-Q256V1, -Q256V13),
    gr_complex(-Q256V1, -Q256V15),
    gr_complex(-Q256V1, Q256V5), gr_complex(-Q256V1, Q256V7), gr_complex(-Q256V1, Q256V3), gr_complex(-Q256V1, Q256V1),
    gr_complex(-Q256V1, Q256V11), gr_complex(-Q256V1, Q256V9), gr_complex(-Q256V1, Q256V13),
    gr_complex(-Q256V1, Q256V15),

    gr_complex(-Q256V3, -Q256V5), gr_complex(-Q256V3, -Q256V7), gr_complex(-Q256V3, -Q256V3),
    gr_complex(-Q256V3, -Q256V1),
    gr_complex(-Q256V3, -Q256V11), gr_complex(-Q256V3, -Q256V9), gr_complex(-Q256V3, -Q256V13),
    gr_complex(-Q256V3, -Q256V15),
    gr_complex(-Q256V3, Q256V5), gr_complex(-Q256V3, Q256V7), gr_complex(-Q256V3, Q256V3), gr_complex(-Q256V3, Q256V1),
    gr_complex(-Q256V3, Q256V11), gr_complex(-Q256V3, Q256V9), gr_complex(-Q256V3, Q256V13),
    gr_complex(-Q256V3, Q256V15),

    gr_complex(-Q256V7, -Q256V5), gr_complex(-Q256V7, -Q256V7), gr_complex(-Q256V7, -Q256V3),
    gr_complex(-Q256V7, -Q256V1),
    gr_complex(-Q256V7, -Q256V11), gr_complex(-Q256V7, -Q256V9), gr_complex(-Q256V7, -Q256V13),
    gr_complex(-Q256V7, -Q256V15),
    gr_complex(-Q256V7, Q256V5), gr_complex(-Q256V7, Q256V7), gr_complex(-Q256V7, Q256V3), gr_complex(-Q256V7, Q256V1),
    gr_complex(-Q256V7, Q256V11), gr_complex(-Q256V7, Q256V9), gr_complex(-Q256V7, Q256V13),
    gr_complex(-Q256V7, Q256V15),

    gr_complex(-Q256V5, -Q256V5), gr_complex(-Q256V5, -Q256V7), gr_complex(-Q256V5, -Q256V3),
    gr_complex(-Q256V5, -Q256V1),
    gr_complex(-Q256V5, -Q256V11), gr_complex(-Q256V5, -Q256V9), gr_complex(-Q256V5, -Q256V13),
    gr_complex(-Q256V5, -Q256V15),
    gr_complex(-Q256V5, Q256V5), gr_complex(-Q256V5, Q256V7), gr_complex(-Q256V5, Q256V3), gr_complex(-Q256V5, Q256V1),
    gr_complex(-Q256V5, Q256V11), gr_complex(-Q256V5, Q256V9), gr_complex(-Q256V5, Q256V13),
    gr_complex(-Q256V5, Q256V15),

    gr_complex(Q256V15, -Q256V5), gr_complex(Q256V15, -Q256V7), gr_complex(Q256V15, -Q256V3),
    gr_complex(Q256V15, -Q256V1),
    gr_complex(Q256V15, -Q256V11), gr_complex(Q256V15, -Q256V9), gr_complex(Q256V15, -Q256V13),
    gr_complex(Q256V15, -Q256V15),
    gr_complex(Q256V15, Q256V5), gr_complex(Q256V15, Q256V7), gr_complex(Q256V15, Q256V3), gr_complex(Q256V15, Q256V1),
    gr_complex(Q256V15, Q256V11), gr_complex(Q256V15, Q256V9), gr_complex(Q256V15, Q256V13),
    gr_complex(Q256V15, Q256V15),

    gr_complex(Q256V13, -Q256V5), gr_complex(Q256V13, -Q256V7), gr_complex(Q256V13, -Q256V3),
    gr_complex(Q256V13, -Q256V1),
    gr_complex(Q256V13, -Q256V11), gr_complex(Q256V13, -Q256V9), gr_complex(Q256V13, -Q256V13),
    gr_complex(Q256V13, -Q256V15),
    gr_complex(Q256V13, Q256V5), gr_complex(Q256V13, Q256V7), gr_complex(Q256V13, Q256V3), gr_complex(Q256V13, Q256V1),
    gr_complex(Q256V13, Q256V11), gr_complex(Q256V13, Q256V9), gr_complex(Q256V13, Q256V13),
    gr_complex(Q256V13, Q256V15),

    gr_complex(Q256V9, -Q256V5), gr_complex(Q256V9, -Q256V7), gr_complex(Q256V9, -Q256V3), gr_complex(Q256V9, -Q256V1),
    gr_complex(Q256V9, -Q256V11), gr_complex(Q256V9, -Q256V9), gr_complex(Q256V9, -Q256V13),
    gr_complex(Q256V9, -Q256V15),
    gr_complex(Q256V9, Q256V5), gr_complex(Q256V9, Q256V7), gr_complex(Q256V9, Q256V3), gr_complex(Q256V9, Q256V1),
    gr_complex(Q256V9, Q256V11), gr_complex(Q256V9, Q256V9), gr_complex(Q256V9, Q256V13), gr_complex(Q256V9, Q256V15),

    gr_complex(Q256V11, -Q256V5), gr_complex(Q256V11, -Q256V7), gr_complex(Q256V11, -Q256V3),
    gr_complex(Q256V11, -Q256V1),
    gr_complex(Q256V11, -Q256V11), gr_complex(Q256V11, -Q256V9), gr_complex(Q256V11, -Q256V13),
    gr_complex(Q256V11, -Q256V15),
    gr_complex(Q256V11, Q256V5), gr_complex(Q256V11, Q256V7), gr_complex(Q256V11, Q256V3), gr_complex(Q256V11, Q256V1),
    gr_complex(Q256V11, Q256V11), gr_complex(Q256V11, Q256V9), gr_complex(Q256V11, Q256V13),
    gr_complex(Q256V11, Q256V15),

    gr_complex(Q256V1, -Q256V5), gr_complex(Q256V1, -Q256V7), gr_complex(Q256V1, -Q256V3), gr_complex(Q256V1, -Q256V1),
    gr_complex(Q256V1, -Q256V11), gr_complex(Q256V1, -Q256V9), gr_complex(Q256V1, -Q256V13),
    gr_complex(Q256V1, -Q256V15),
    gr_complex(Q256V1, Q256V5), gr_complex(Q256V1, Q256V7), gr_complex(Q256V1, Q256V3), gr_complex(Q256V1, Q256V1),
    gr_complex(Q256V1, Q256V11), gr_complex(Q256V1, Q256V9), gr_complex(Q256V1, Q256V13), gr_complex(Q256V1, Q256V15),

    gr_complex(Q256V3, -Q256V5), gr_complex(Q256V3, -Q256V7), gr_complex(Q256V3, -Q256V3), gr_complex(Q256V3, -Q256V1),
    gr_complex(Q256V3, -Q256V11), gr_complex(Q256V3, -Q256V9), gr_complex(Q256V3, -Q256V13),
    gr_complex(Q256V3, -Q256V15),
    gr_complex(Q256V3, Q256V5), gr_complex(Q256V3, Q256V7), gr_complex(Q256V3, Q256V3), gr_complex(Q256V3, Q256V1),
    gr_complex(Q256V3, Q256V11), gr_complex(Q256V3, Q256V9), gr_complex(Q256V3, Q256V13), gr_complex(Q256V3, Q256V15),

    gr_complex(Q256V7, -Q256V5), gr_complex(Q256V7, -Q256V7), gr_complex(Q256V7, -Q256V3), gr_complex(Q256V7, -Q256V1),
    gr_complex(Q256V7, -Q256V11), gr_complex(Q256V7, -Q256V9), gr_complex(Q256V7, -Q256V13),
    gr_complex(Q256V7, -Q256V15),
    gr_complex(Q256V7, Q256V5), gr_complex(Q256V7, Q256V7), gr_complex(Q256V7, Q256V3), gr_complex(Q256V7, Q256V1),
    gr_complex(Q256V7, Q256V11), gr_complex(Q256V7, Q256V9), gr_complex(Q256V7, Q256V13), gr_complex(Q256V7, Q256V15),

    gr_complex(Q256V5, -Q256V5), gr_complex(Q256V5, -Q256V7), gr_complex(Q256V5, -Q256V3), gr_complex(Q256V5, -Q256V1),
    gr_complex(Q256V5, -Q256V11), gr_complex(Q256V5, -Q256V9), gr_complex(Q256V5, -Q256V13),
    gr_complex(Q256V5, -Q256V15),
    gr_complex(Q256V5, Q256V5), gr_complex(Q256V5, Q256V7), gr_complex(Q256V5, Q256V3), gr_complex(Q256V5, Q256V1),
    gr_complex(Q256V5, Q256V11), gr_complex(Q256V5, Q256V9), gr_complex(Q256V5, Q256V13), gr_complex(Q256V5, Q256V15)
};

} /* namespace gr::ncjt */
