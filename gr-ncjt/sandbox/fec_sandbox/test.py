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

p1mod = 6
p2mod = 4
p3mod = 8
code_rate = 3
deterministic = True

frames_per_sec = 5
SNR = 0

rgmode = 2
num_streams = 1


import random
tb = gr.top_block()


## Phase 1
mapper_muxer_debug = False
ncjt_mapper_muxer = ncjt.mapper_muxer(rgmode, num_streams, p1mod, p2mod, p3mod, code_rate, deterministic, mapper_muxer_debug)

blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, ncjt.RG_NUM_OFDM_SYM[rgmode] * ncjt.RG_NUM_DATA_SC[rgmode])

rg_mapper_0_debug = False
ncjt_rg_mapper_0_1 = ncjt.rg_mapper(rgmode, num_streams, False, rg_mapper_0_debug, 1, (-1), False, False, '')

ncjt_noair_0_0 = ncjt.noair(rgmode, frames_per_sec, SNR, 0, False)

rg_demapper_0_debug = True
rg_demapper_0_phase = 1
ncjt_rg_demapper_0 = ncjt.rg_demapper(rg_demapper_0_phase, rgmode, num_streams, True, True, rg_demapper_0_debug)

demapper_0_debug = False
ncjt_demapper_0 = ncjt.demapper(rgmode, False, deterministic, demapper_0_debug)

remapper_0_debug = False
ncjt_remapper_muxer_0_0_0 = ncjt.remapper_muxer(rgmode, num_streams, remapper_0_debug)

## Phase 2
rg_mapper_1_debug = False
ncjt_rg_mapper_1 = ncjt.rg_mapper(rgmode, num_streams, False, rg_mapper_1_debug, 1, (-1), False, False, '')

ncjt_noair_1 = ncjt.noair(rgmode, frames_per_sec, SNR, 0, False)

rg_demapper_1_debug = False
rg_demapper_1_phase = 2
ncjt_rg_demapper_1 = ncjt.rg_demapper(rg_demapper_1_phase, rgmode, num_streams, True, True, rg_demapper_1_debug)

demapper_1_debug = False
ncjt_demapper_1 = ncjt.demapper(rgmode, False, deterministic, demapper_1_debug)

remapper_1_debug = False
ncjt_remapper_muxer_1_0_0 = ncjt.remapper_muxer(rgmode, num_streams, remapper_1_debug)



complex_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
char_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_char*1)


tb.connect((ncjt_mapper_muxer, 0), (blocks_head_0, 0))
tb.connect((blocks_head_0, 0), (ncjt_rg_mapper_0_1, 0))
tb.connect((ncjt_rg_mapper_0_1, 0), (ncjt_noair_0_0, 0))

# Tx gNB -> Tx UE
tb.connect((ncjt_noair_0_0, 0), (ncjt_rg_demapper_0, 0))
tb.connect((ncjt_noair_0_0, 1), (ncjt_rg_demapper_0, 1))

tb.connect((ncjt_rg_demapper_0, 0), (ncjt_demapper_0, 0))
tb.connect((ncjt_rg_demapper_0, 1), (ncjt_demapper_0, 1))

# First remapper
tb.connect((ncjt_demapper_0, 0), (ncjt_remapper_muxer_0_0_0, 0))

tb.connect((ncjt_remapper_muxer_0_0_0, 0), (ncjt_rg_mapper_1, 0))

# TX UE -> RX UE
tb.connect((ncjt_rg_mapper_1, 0), (ncjt_noair_1, 0))

tb.connect((ncjt_noair_1, 0), (ncjt_rg_demapper_1, 0))
tb.connect((ncjt_noair_1, 1), (ncjt_rg_demapper_1, 1))

tb.connect((ncjt_rg_demapper_1, 0), (ncjt_demapper_1, 0))
tb.connect((ncjt_rg_demapper_1, 1), (ncjt_demapper_1, 1))

# Second remapper
tb.connect((ncjt_demapper_1, 0), (ncjt_remapper_muxer_1_0_0, 0))

tb.connect((ncjt_remapper_muxer_1_0_0, 0), (complex_blocks_null_sink_1, 0))


tb.run()