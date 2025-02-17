/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/pybind11.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace py = pybind11;

// Headers for binding functions
/**************************************/
// The following comment block is used for
// gr_modtool to insert function prototypes
// Please do not delete
/**************************************/
// BINDING_FUNCTION_PROTOTYPES(
    void bind_burst_tx(py::module& m);
    void bind_pkt_detect(py::module& m);
    void bind_sync_all(py::module& m);
    void bind_ofdm_mod(py::module& m);
    void bind_ofdm_demod(py::module& m);
    void bind_ltf_chanest(py::module& m);
    void bind_qam_demapper(py::module& m);
    void bind_soft_demapper(py::module& m);
    void bind_pkt_err(py::module& m);
    void bind_tx_framing(py::module& m);
    void bind_rg_mapper(py::module& m);
    void bind_rg_demapper(py::module& m);
    void bind_ul_precoding(py::module& m);
    void bind_mimo_detect(py::module& m);
    void bind_rx_mrc(py::module& m);
    void bind_mu_chanest(py::module& m);
    void bind_sic_detect(py::module& m);
    void bind_stbc_encode(py::module& m);
    void bind_stbc_decode(py::module& m);
    void bind_add_preamble(py::module& m);
// ) END BINDING_FUNCTION_PROTOTYPES


// We need this hack because import_array() returns NULL
// for newer Python versions.
// This function is also necessary because it ensures access to the C API
// and removes a warning.
void* init_numpy()
{
    import_array();
    return NULL;
}

PYBIND11_MODULE(ncjt_python, m)
{
    // Initialize the numpy C API
    // (otherwise we will see segmentation faults)
    init_numpy();

    // Allow access to base block methods
    py::module::import("gnuradio.gr");

    /**************************************/
    // The following comment block is used for
    // gr_modtool to insert binding function calls
    // Please do not delete
    /**************************************/
    // BINDING_FUNCTION_CALLS(
    bind_burst_tx(m);
    bind_pkt_detect(m);
    bind_sync_all(m);
    bind_ofdm_mod(m);
    bind_ofdm_demod(m);
    bind_ltf_chanest(m);
    bind_qam_demapper(m);
    bind_soft_demapper(m);
    bind_pkt_err(m);
    bind_tx_framing(m);
    bind_rg_mapper(m);
    bind_rg_demapper(m);
    bind_ul_precoding(m);
    bind_mimo_detect(m);
    bind_rx_mrc(m);
    bind_mu_chanest(m);
    bind_sic_detect(m);
    bind_stbc_encode(m);
    bind_stbc_decode(m);
    bind_add_preamble(m);
    // ) END BINDING_FUNCTION_CALLS
}