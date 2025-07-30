# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT ncjt_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT ncjt_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################
list(APPEND ncjt_python_files
    burst_tx_python.cc
    ofdm_mod_python.cc
    ofdm_demod_python.cc
    ltf_chanest_python.cc
    pkt_err_python.cc
    rg_mapper_python.cc
    rg_demapper_python.cc
    ul_precoding_python.cc
    mimo_detect_python.cc
    mu_chanest_python.cc
    sic_detect_python.cc
    stbc_encode_python.cc
    stbc_decode_python.cc
    mc_stbc_decode_python.cc
    srsran_ldpc_encoder_python.cc
    srsran_ldpc_decoder_python.cc
    video_source_python.cc
    video_sink_python.cc
    rx_sync_python.cc
    gnb_sync_python.cc
    tx_frm_ctrl_python.cc
    dual_tx_ctrl_python.cc
    mapper_muxer_python.cc
    noair_python.cc
    demapper_python.cc
    remapper_muxer_python.cc
    rg_modes_python.cc
    pdc_python.cc
    skip_data_python.cc
    csi_quantizer_python.cc
    csi_fb_processing_python.cc
    p1_precoding_python.cc
    stream_mapper_python.cc python_bindings.cc)

gr_pybind_make_oot(ncjt ../../.. gr::ncjt "${ncjt_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET ncjt_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ncjt_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/ncjt/)

install(
    TARGETS ncjt_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/ncjt
    COMPONENT pythonapi)
