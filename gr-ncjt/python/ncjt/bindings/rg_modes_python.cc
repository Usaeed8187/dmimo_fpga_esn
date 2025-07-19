#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <gnuradio/ncjt/rg_modes.h>

namespace py = pybind11;


void bind_rg_modes(py::module &m)
{
    m.attr("NUM_RG_MODES") = NUM_RG_MODES;
    m.attr("MAX_NUM_CPT")  = MAX_NUM_CPT;

    // FFT size
    py::list fft_size_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        fft_size_list.append(RG_FFT_SIZE[i]);
    m.attr("RG_FFT_SIZE") = fft_size_list;

    // valid subcarriers
    py::list valid_sc_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        valid_sc_list.append(RG_NUM_VALID_SC[i]);
    m.attr("RG_NUM_VALID_SC") = valid_sc_list;

    // data subcarriers
    py::list data_sc_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        data_sc_list.append(RG_NUM_DATA_SC[i]);
    m.attr("RG_NUM_DATA_SC") = data_sc_list;

    // guard subcarriers
    py::list guard_sc_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        guard_sc_list.append(RG_NUM_GUARD_SC[i]);
    m.attr("RG_NUM_GUARD_SC") = guard_sc_list;

    // continuous pilot tone count
    py::list cpt_count_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        cpt_count_list.append(RG_NUM_CPT[i]);
    m.attr("RG_NUM_CPT") = cpt_count_list;

    // CPT indices
    py::list cpt_indices_list;
    for (int i = 0; i < NUM_RG_MODES; i++) {
        py::list one_mode;
        for (int j = 0; j < MAX_NUM_CPT; j++)
            one_mode.append(RG_CPT_INDX[i][j]);
        cpt_indices_list.append(one_mode);
    }
    m.attr("RG_CPT_INDX") = cpt_indices_list;

    // OFDM symbols
    py::list ofdm_sym_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        ofdm_sym_list.append(RG_NUM_OFDM_SYM[i]);
    m.attr("RG_NUM_OFDM_SYM") = ofdm_sym_list;

    // DMRS symbols
    py::list dmrs_sym_list;
    for (int i = 0; i < NUM_RG_MODES; i++)
        dmrs_sym_list.append(RG_NUM_DMRS_SYM[i]);
    m.attr("RG_NUM_DMRS_SYM") = dmrs_sym_list;
}