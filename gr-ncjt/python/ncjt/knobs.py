#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 Wireless@VT.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy
from PyQt5 import Qt, QtWidgets
from gnuradio import gr
import pmt

class knobs(gr.sync_block, QtWidgets.QWidget):
    """
    docstring for block knobs
    """
    def __init__(self, mcs, phase1_modtype, phase2_modtype, phase3_modtype, code_rate, reset, noair):
        gr.sync_block.__init__(self,
            name="knobs",
            in_sig=[],
            out_sig=[])
        ###
        Qt.QWidget.__init__(self)
        self.mainLayout = Qt.QGridLayout()
        self.setLayout(self.mainLayout)
        self.setStyleSheet("font-size: 16pt;")  # Increase font size for all UI elements
        row = 0
        ###
        if mcs:
            self.mainLayout.addWidget(Qt.QLabel('Phase1 Mod:'), row, 0)
            self.mod_type_phase1 = Qt.QComboBox()
            self.mainLayout.addWidget(self.mod_type_phase1, row, 1)
            row += 1
            #
            self.mainLayout.addWidget(Qt.QLabel('Phase2 Mod:'), row, 0)
            self.mod_type_phase2 = Qt.QComboBox()
            self.mainLayout.addWidget(self.mod_type_phase2, row, 1)
            row += 1
            #
            self.mainLayout.addWidget(Qt.QLabel('Phase3 Mod:'), row, 0)
            self.mod_type_phase3 = Qt.QComboBox()
            self.mainLayout.addWidget(self.mod_type_phase3, row, 1)
            row += 1
            #
            modtype_to_ind = {2: 0, 4: 1, 6: 2, 8: 3}
            for i, mt_widget in enumerate([self.mod_type_phase1, self.mod_type_phase2, self.mod_type_phase3]):
                for mtype in ['QPSK', '16-QAM', '64-QAM', '256-QAM']:
                    mt_widget.addItem(mtype)
                mt = [phase1_modtype, phase2_modtype, phase3_modtype][i]
                mt_widget.setCurrentIndex(modtype_to_ind[mt])
                mt_widget.currentIndexChanged.connect(self.onSelModTypeChange)
            # self.mod_type_phase1.addItem('QPSK')
            # self.mod_type_phase1.addItem('16-QAM')
            # self.mod_type_phase1.addItem('64-QAM')
            # self.mod_type_phase1.addItem('64-QAM')
            # self.mod_type_phase1.addItem('256-QAM')
            # self.mod_type_phase1.setCurrentIndex(0)
            # self.mod_type_phase1.currentIndexChanged.connect(self.onSelModTypeChange)

            self.message_port_register_out(pmt.intern('s_modtypes'))
            ##


            # Add filler to third column
            self.mainLayout.addWidget(Qt.QLabel(''), 0, 2)
            ###
            self.mainLayout.addWidget(Qt.QLabel('Code Rate:'), row, 0)
            self.code_rate = Qt.QComboBox()
            code_rate_items = [
                'No LDPC Coding',
                '1/4',
                '1/3',
                '1/2',
                '2/3',
                '3/4',
                '4/5',
                '5/6'
            ]
            for item in code_rate_items:
                self.code_rate.addItem(item)
            self.code_rate.setCurrentIndex(code_rate)
            self.code_rate.currentIndexChanged.connect(self.onSelCodeRateChange)
            self.mainLayout.addWidget(self.code_rate, row, 1)
            self.message_port_register_out(pmt.intern('s_code_rate'))
            row += 1
        ###
        if noair:
            self.mainLayout.addWidget(Qt.QLabel('SNR1 (dB):'), row, 0)
            #
            self.snr1 = Qt.QLineEdit()
            self.snr1.setText('20')
            self.snr1.editingFinished.connect(self.onSNR1Change)
            self.mainLayout.addWidget(self.snr1, row, 1)
            self.message_port_register_out(pmt.intern('s_snr1'))
            row += 1
            # SNR2
            self.mainLayout.addWidget(Qt.QLabel('SNR2 (dB):'), row, 0)
            self.snr2 = Qt.QLineEdit()
            self.snr2.setText('20')
            self.snr2.editingFinished.connect(self.onSNR2Change)
            self.mainLayout.addWidget(self.snr2, row, 1)
            self.message_port_register_out(pmt.intern('s_snr2'))
            row += 1
            # SNR3
            self.mainLayout.addWidget(Qt.QLabel('SNR3 (dB):'), row, 0)
            self.snr3 = Qt.QLineEdit()
            self.snr3.setText('20')
            self.snr3.editingFinished.connect(self.onSNR3Change)
            self.mainLayout.addWidget(self.snr3, row, 1)
            self.message_port_register_out(pmt.intern('s_snr3'))
            row += 1

        ###
        if reset:
            self.reset_button = Qt.QPushButton('Reset Statistics')
            self.reset_button.setStyleSheet("font-size: 16pt;")
            self.reset_button.clicked.connect(self.onResetButtonClick)
            self.mainLayout.addWidget(self.reset_button, row, 2)
            self.message_port_register_out(pmt.intern('reset'))
            row += 1
        ###
        # I want third column to expand
        self.mainLayout.setColumnStretch(2, 1)

    def onResetButtonClick(self):
        print('[knobs::onResetButtonClick] Reset Statistics')
        self.message_port_pub(pmt.intern("reset"), pmt.to_pmt(True))

    def onSelModTypeChange(self):
        def get_modtype(combo):
            t = combo.currentText()
            if t == 'QPSK':
                return 2
            elif t == '16-QAM':
                return 4
            elif t == '64-QAM':
                return 6
            elif t == '256-QAM':
                return 8
            else:
                raise Exception(f'Unknown modulation type: {t}')
        mtypes = [2, 4, 6, 8]
        p1 = get_modtype(self.mod_type_phase1)
        p2 = get_modtype(self.mod_type_phase2)
        p3 = get_modtype(self.mod_type_phase3)
        ##
        # if p2 < p1:
        #     p2 = p1
        #     self.mod_type_phase2.blockSignals(True)
        #     self.mod_type_phase2.setCurrentIndex(mtypes.index(p2))
        #     self.mod_type_phase2.blockSignals(False)
        # if p3 < p2:
        #     p3 = p2
        #     self.mod_type_phase3.blockSignals(True)
        #     self.mod_type_phase3.setCurrentIndex(mtypes.index(p3))
        #     self.mod_type_phase3.blockSignals(False)

        # Ensure p1 and p3 are not less than p2
        if p1 < p2:
            p1 = p2
            self.mod_type_phase1.blockSignals(True)
            self.mod_type_phase1.setCurrentIndex(mtypes.index(p1))
            self.mod_type_phase1.blockSignals(False)
        if p3 < p2:
            p3 = p2
            self.mod_type_phase3.blockSignals(True)
            self.mod_type_phase3.setCurrentIndex(mtypes.index(p3))
            self.mod_type_phase3.blockSignals(False)


        modtype = p3
        modtype |= p2 << 4
        modtype |= p1 << 8

        self.message_port_pub(pmt.intern("s_modtypes"), pmt.to_pmt(modtype))



        log_msg = f'[knobs::onSelModTypeChange]'
        log_msg += f'P1: {self.mod_type_phase1.currentText()}, '
        log_msg += f'P2: {self.mod_type_phase2.currentText()}, '
        log_msg += f'P3: {self.mod_type_phase3.currentText()}'
        print(log_msg)

    def onSelCodeRateChange(self):
        print(f'[knobs::onSelCodeRateChange] Code Rate: {self.code_rate.currentText()}')
        coderate = None
        if self.code_rate.currentText() == 'No LDPC Coding':
            coderate = 0
        elif self.code_rate.currentText() == '1/4':
            coderate = 1
        elif self.code_rate.currentText() == '1/3':
            coderate = 2
        elif self.code_rate.currentText() == '1/2':
            coderate = 3
        elif self.code_rate.currentText() == '2/3':
            coderate = 4
        elif self.code_rate.currentText() == '3/4':
            coderate = 5
        elif self.code_rate.currentText() == '4/5':
            coderate = 6
        elif self.code_rate.currentText() == '5/6':
            coderate = 7
        self.message_port_pub(pmt.intern("s_code_rate"), pmt.to_pmt(coderate))

    def onSNR1Change(self):
        print(f'[knobs::onSNR1Change] SNR1: {self.snr1.text()}')
        self.message_port_pub(pmt.intern("s_snr1"), pmt.to_pmt(float(self.snr1.text())))

    def onSNR2Change(self):
        print(f'[knobs::onSNR2Change] SNR2: {self.snr2.text()}')
        self.message_port_pub(pmt.intern("s_snr2"), pmt.to_pmt(float(self.snr2.text())))
    def onSNR3Change(self):
        print(f'[knobs::onSNR3Change] SNR3: {self.snr3.text()}')
        self.message_port_pub(pmt.intern("s_snr3"), pmt.to_pmt(float(self.snr3.text())))


    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        # <+signal processing here+>
        out[:] = in0
        return len(output_items[0])
