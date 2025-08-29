import os
os.system('cls' if os.name == 'nt' else 'clear')

import code
from math import floor
import os
import time
import argparse
import sys

import numpy as np

from gnuradio import gr
from gnuradio import ncjt
from gnuradio import blocks

p1mod = 4
p2mod = 4
p3mod = 4
code_rate = 1
deterministic = True

frames_per_sec = 5
SNR = 0

rgmode = 0
num_streams = 1

enable_pdc = True

debug_all = True


import random
tb = gr.top_block()


## Phase 1
mapper_muxer_debug = False or debug_all
ncjt_mapper_muxer_phase1 = ncjt.mapper_muxer(rgmode, num_streams, p1mod, p2mod, p3mod, code_rate, deterministic, mapper_muxer_debug)

blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, ncjt.RG_NUM_OFDM_SYM[rgmode] * ncjt.RG_NUM_DATA_SC[rgmode])

rg_mapper_phase1_debug = False or debug_all
ncjt_rg_mapper_phase1 = ncjt.rg_mapper(rgmode, num_streams, False, rg_mapper_phase1_debug, 1, (-1), False, False, '')

ncjt_noair_1st_hop = ncjt.noair(1, rgmode, frames_per_sec, SNR, 0, False)

rg_demapper_phase1_debug = False or debug_all
rg_demapper_phase1_phase = 1
ncjt_rg_demapper_phase1 = ncjt.rg_demapper(rg_demapper_phase1_phase, rgmode, num_streams, True, rg_demapper_phase1_debug)

demapper_phase1_debug = False or debug_all
ncjt_demapper_phase1 = ncjt.demapper(rgmode, False, deterministic, 0, demapper_phase1_debug)

remapper_phase1_debug = False or debug_all
remapper_phase1_reencode = True
ncjt_remapper_muxer_phase1 = ncjt.remapper_muxer(rgmode, num_streams, remapper_phase1_reencode, remapper_phase1_debug)

## Phase 2
rg_mapper_phase2_debug = False or debug_all
ncjt_rg_mapper_phase2 = ncjt.rg_mapper(rgmode, num_streams, False, rg_mapper_phase2_debug, 1, (-1), False, False, '')

ncjt_noair_2nd_hop = ncjt.noair(2, rgmode, frames_per_sec, SNR, 0, False)

rg_demapper_phase2_debug = False or debug_all
rg_demapper_phase2_phase = 2
ncjt_rg_demapper_phase2 = ncjt.rg_demapper(rg_demapper_phase2_phase, rgmode, num_streams, True, rg_demapper_phase2_debug)

demapper_phase2_debug = False or debug_all
ncjt_demapper_phase2 = ncjt.demapper(rgmode, False, deterministic, 0, demapper_phase2_debug)

remapper_phase2_debug = False or debug_all
remapper_phase2_reencode = False
ncjt_remapper_muxer_phase2 = ncjt.remapper_muxer(rgmode, num_streams, remapper_phase2_reencode, remapper_phase2_debug)

## Phase 3
ncopies = 3

ncjt_rg_mapper_phase3_1 = ncjt.rg_mapper(rgmode, num_streams, False, False, 2, 2, True, False, '')
##
ncjt_noair_3rd_hop_1 = ncjt.noair(3, rgmode, frames_per_sec, SNR, 0, False)
##
rg_demapper_phase3_1 = False or debug_all
rg_demapper_phase3_1_phase = 3
ncjt_rg_demapper_phase3_1 = ncjt.rg_demapper(rg_demapper_phase3_1_phase, rgmode, num_streams, True, rg_demapper_phase3_1)

if enable_pdc:
    ncjt_rg_mapper_phase3_2 = ncjt.rg_mapper(rgmode, num_streams, False, False, 2, 1, True, False, '')
    ##
    ncjt_noair_3rd_hop_2 = ncjt.noair(3, rgmode, frames_per_sec, SNR, 0, False)
    ##
    rg_demapper_phase3_2 = False or debug_all
    rg_demapper_phase3_2_phase = 3
    ncjt_rg_demapper_phase3_2 = ncjt.rg_demapper(rg_demapper_phase3_2_phase, rgmode, num_streams, True, rg_demapper_phase3_2)

if enable_pdc:
    pdc_debug = False or debug_all
    ncjt_pdc_0 = ncjt.pdc(rgmode, True, ncopies, 100, 0, deterministic, 0, 0, pdc_debug)
else:
    demapper_phase3_debug = True or debug_all
    ncjt_demapper_phase3 = ncjt.demapper(rgmode, False, deterministic, 0, demapper_phase3_debug)


complex_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_2 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_3 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_4 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_5 = blocks.null_sink(gr.sizeof_gr_complex*1)
char_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_char*1)
char_blocks_null_sink_1 = blocks.null_sink(gr.sizeof_char*1)


tb.connect((ncjt_mapper_muxer_phase1, 0), (blocks_head_0, 0))
tb.connect((blocks_head_0, 0), (ncjt_rg_mapper_phase1, 0))
tb.connect((ncjt_rg_mapper_phase1, 0), (ncjt_noair_1st_hop, 0))

# Tx gNB -> Tx UE
tb.connect((ncjt_noair_1st_hop, 0), (ncjt_rg_demapper_phase1, 0))
tb.connect((ncjt_noair_1st_hop, 1), (ncjt_rg_demapper_phase1, 1))

tb.connect((ncjt_rg_demapper_phase1, 0), (ncjt_demapper_phase1, 0))
tb.connect((ncjt_rg_demapper_phase1, 1), (ncjt_demapper_phase1, 1))

# tb.connect((ncjt_demapper_phase1, 0), (char_blocks_null_sink_0, 0))

# First remapper
tb.connect((ncjt_demapper_phase1, 0), (ncjt_remapper_muxer_phase1, 0))

tb.connect((ncjt_remapper_muxer_phase1, 0), (ncjt_rg_mapper_phase2, 0))

# TX UE -> RX UE
tb.connect((ncjt_rg_mapper_phase2, 0), (ncjt_noair_2nd_hop, 0))

tb.connect((ncjt_noair_2nd_hop, 0), (ncjt_rg_demapper_phase2, 0))
tb.connect((ncjt_noair_2nd_hop, 1), (ncjt_rg_demapper_phase2, 1))

tb.connect((ncjt_rg_demapper_phase2, 0), (ncjt_demapper_phase2, 0))
tb.connect((ncjt_rg_demapper_phase2, 1), (ncjt_demapper_phase2, 1))

# tb.connect((ncjt_demapper_phase2, 0), (char_blocks_null_sink_0, 0))

# Second remapper
tb.connect((ncjt_demapper_phase2, 0), (ncjt_remapper_muxer_phase2, 0))

tb.connect((ncjt_remapper_muxer_phase2, 0), (ncjt_rg_mapper_phase3_1, 0))
if enable_pdc:
    tb.connect((ncjt_remapper_muxer_phase2, 0), (ncjt_rg_mapper_phase3_2, 0))

# Rx UE -> Rx gNB
tb.connect((ncjt_rg_mapper_phase3_1, 0), (ncjt_noair_3rd_hop_1, 0))
##
tb.connect((ncjt_noair_3rd_hop_1, 0), (ncjt_rg_demapper_phase3_1, 0))
tb.connect((ncjt_noair_3rd_hop_1, 1), (ncjt_rg_demapper_phase3_1, 1))

if enable_pdc:
    tb.connect((ncjt_rg_mapper_phase3_2, 0), (ncjt_noair_3rd_hop_2, 0))
    ##
    tb.connect((ncjt_noair_3rd_hop_2, 0), (ncjt_rg_demapper_phase3_2, 0))
    tb.connect((ncjt_noair_3rd_hop_2, 1), (ncjt_rg_demapper_phase3_2, 1))

    if ncopies == 1:
        tb.connect((ncjt_rg_demapper_phase3_1, 0), (ncjt_pdc_0, 0))
        tb.connect((ncjt_rg_demapper_phase3_1, 1), (ncjt_pdc_0, 1))
        tb.connect((ncjt_rg_demapper_phase3_2, 0), (complex_blocks_null_sink_0, 0))
        tb.connect((ncjt_rg_demapper_phase3_2, 1), (complex_blocks_null_sink_1, 0))
        tb.connect((ncjt_rg_demapper_phase2, 0), (complex_blocks_null_sink_2, 0))
        tb.connect((ncjt_rg_demapper_phase2, 1), (complex_blocks_null_sink_3, 0))
    elif ncopies == 2:
        tb.connect((ncjt_rg_demapper_phase3_1, 0), (ncjt_pdc_0, 0))
        tb.connect((ncjt_rg_demapper_phase3_1, 1), (ncjt_pdc_0, 2))
        tb.connect((ncjt_rg_demapper_phase3_2, 0), (ncjt_pdc_0, 1))
        tb.connect((ncjt_rg_demapper_phase3_2, 1), (ncjt_pdc_0, 3))
        tb.connect((ncjt_rg_demapper_phase2, 0), (complex_blocks_null_sink_0, 0))
        tb.connect((ncjt_rg_demapper_phase2, 1), (complex_blocks_null_sink_1, 0))
    elif ncopies == 3:
        tb.connect((ncjt_rg_demapper_phase3_1, 0), (ncjt_pdc_0, 0))
        tb.connect((ncjt_rg_demapper_phase3_1, 1), (ncjt_pdc_0, 3))
        tb.connect((ncjt_rg_demapper_phase3_2, 0), (ncjt_pdc_0, 1))
        tb.connect((ncjt_rg_demapper_phase3_2, 1), (ncjt_pdc_0, 4))
        tb.connect((ncjt_rg_demapper_phase2, 0), (ncjt_pdc_0, 2))
        tb.connect((ncjt_rg_demapper_phase2, 1), (ncjt_pdc_0, 5))
    tb.connect((ncjt_pdc_0, 0), (char_blocks_null_sink_0, 0))
    tb.connect((ncjt_pdc_0, 1), (char_blocks_null_sink_1, 0))
else:
    tb.connect((ncjt_rg_demapper_phase3_1, 0), (ncjt_demapper_phase3, 0))
    tb.connect((ncjt_rg_demapper_phase3_1, 1), (ncjt_demapper_phase3, 1))
    tb.connect((ncjt_demapper_phase3, 0), (char_blocks_null_sink_0, 0))


tb.run()