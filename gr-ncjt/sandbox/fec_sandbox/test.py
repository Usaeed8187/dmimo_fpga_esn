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

rgmode = 0
num_streams = 1


import random
tb = gr.top_block()



mapper_muxer_debug = False
ncjt_mapper_muxer = ncjt.mapper_muxer(rgmode, num_streams, p1mod, p2mod, p3mod, code_rate, deterministic, mapper_muxer_debug)

blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, 2080)

rg_mapper_debug = False
ncjt_rg_mapper_0_1 = ncjt.rg_mapper(rgmode, num_streams, False, rg_mapper_debug, 1, (-1), False, False, '')

ncjt_noair_0_0 = ncjt.noair(rgmode, frames_per_sec, SNR, 0, False)

rg_demapper_debug = False
rg_demapper_0_phase = 1
ncjt_rg_demapper_0 = ncjt.rg_demapper(rg_demapper_0_phase, rgmode, num_streams, True, True, rg_demapper_debug)

demapper_0_debug = True
ncjt_demapper_0 = ncjt.demapper(rgmode, False, deterministic, demapper_0_debug)

complex_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
complex_blocks_null_sink_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
char_blocks_null_sink_0 = blocks.null_sink(gr.sizeof_char*1)

tb.connect((ncjt_mapper_muxer, 0), (blocks_head_0, 0))
tb.connect((blocks_head_0, 0), (ncjt_rg_mapper_0_1, 0))
tb.connect((ncjt_rg_mapper_0_1, 0), (ncjt_noair_0_0, 0))

tb.connect((ncjt_noair_0_0, 0), (ncjt_rg_demapper_0, 0))
tb.connect((ncjt_noair_0_0, 1), (ncjt_rg_demapper_0, 1))

tb.connect((ncjt_rg_demapper_0, 0), (ncjt_demapper_0, 0))
tb.connect((ncjt_rg_demapper_0, 1), (ncjt_demapper_0, 1))

tb.connect((ncjt_demapper_0, 0), (char_blocks_null_sink_0, 0))

tb.run()