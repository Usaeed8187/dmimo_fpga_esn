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
    def __init__(self, mcs, reset, noair):
        gr.sync_block.__init__(self,
            name="knobs",
            in_sig=[],
            out_sig=[])
        ###
        Qt.QWidget.__init__(self)
        self.mainLayout = Qt.QGridLayout()
        self.setLayout(self.mainLayout)
        self.setStyleSheet("font-size: 16pt;")  # Increase font size for all UI elements
        ###
        if mcs:
            self.mainLayout.addWidget(Qt.QLabel('Mod Type:'), 0, 0)
            self.mod_type = Qt.QComboBox()
            self.mod_type.addItem('QPSK')
            self.mod_type.addItem('16-QAM')
            self.mod_type.addItem('64-QAM')
            self.mod_type.addItem('256-QAM')
            self.mod_type.setCurrentIndex(0)
            self.mod_type.currentIndexChanged.connect(self.onSelModTypeChange)
            self.mainLayout.addWidget(self.mod_type, 0, 1)
            self.message_port_register_out(pmt.intern('sel_mod_type'))
            # Add filler to third column
            self.mainLayout.addWidget(Qt.QLabel(''), 0, 2)
            ###
            self.mainLayout.addWidget(Qt.QLabel('Code Rate:'), 1, 0)
            self.code_rate = Qt.QComboBox()
            self.code_rate.addItem('No LDPC Coding')
            self.code_rate.addItem('1/4')
            self.code_rate.addItem('1/3')
            self.code_rate.addItem('1/2')
            self.code_rate.addItem('2/3')
            self.code_rate.addItem('3/4')
            self.code_rate.addItem('4/5')
            self.code_rate.addItem('5/6')
            self.code_rate.setCurrentIndex(0)
            self.code_rate.currentIndexChanged.connect(self.onSelCodeRateChange)
            self.mainLayout.addWidget(self.code_rate, 1, 1)
            self.message_port_register_out(pmt.intern('sel_code_rate'))
        ###
        if noair:
            self.mainLayout.addWidget(Qt.QLabel('SNR (dB):'), 2, 0)
            self.snr = Qt.QLineEdit()
            self.snr.setText('20')
            self.snr.editingFinished.connect(self.onSNRChange)
            self.mainLayout.addWidget(self.snr, 2, 1)
            self.message_port_register_out(pmt.intern('sel_snr'))
        ###
        if reset:
            self.reset_button = Qt.QPushButton('Reset Statistics')
            self.reset_button.setStyleSheet("font-size: 16pt;")
            self.reset_button.clicked.connect(self.onResetButtonClick)
            self.mainLayout.addWidget(self.reset_button, 3, 2)
            self.message_port_register_out(pmt.intern('reset'))
        ###
        # I want third column to expand
        self.mainLayout.setColumnStretch(2, 1)

    def onResetButtonClick(self):
        print('[knobs::onResetButtonClick] Reset Statistics')
        self.message_port_pub(pmt.intern("reset"), pmt.to_pmt(True))

    def onSelModTypeChange(self):
        print(f'[knobs::onSelModTypeChange] Mod Type: {self.mod_type.currentText()}')
        modtype = None
        if self.mod_type.currentText() == 'QPSK':
            modtype = 2
        elif self.mod_type.currentText() == '16-QAM':
            modtype = 4
        elif self.mod_type.currentText() == '64-QAM':
            modtype = 6
        elif self.mod_type.currentText() == '256-QAM':
            modtype = 8
        self.message_port_pub(pmt.intern("sel_mod_type"), pmt.to_pmt(modtype))
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
        self.message_port_pub(pmt.intern("sel_code_rate"), pmt.to_pmt(coderate))
    def onSNRChange(self):
        print(f'[knobs::onSNRChange] SNR: {self.snr.text()}')
        self.message_port_pub(pmt.intern("sel_snr"), pmt.to_pmt(float(self.snr.text())))

    def work(self, input_items, output_items):
        print(f'[knobs::work] input_items: {input_items}, output_items: {output_items}')
        return len(output_items[0])



    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        # <+signal processing here+>
        out[:] = in0
        return len(output_items[0])
