#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 Wireless@VT.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from PyQt5.QtCore import Qt  # Import Qt from QtCore for alignment flags
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis, QLogValueAxis
from PyQt5.QtCore import QTimer
from gnuradio import gr
import pmt
import time
import math

EPS = 1e-10

class err_stats(gr.sync_block, QtWidgets.QWidget):
    def __init__(
        self,
        caption,
        cols,
        row_height,
        frame_per_sec,
        logfreq,
        show_bitrate,
        show_throughput,
        show_per,
        show_num_copies,
        show_coded_ber,
        show_uncoded_ber,
        dark,
        debug,
    ):
        if debug:
            print(
                f"[err_stats::__init__] caption: {caption}, frame_per_sec: {frame_per_sec}, logfreq: {logfreq}, show_bitrate: {show_bitrate}, show_throughput: {show_throughput}, show_per: {show_per}, show_num_copies: {show_num_copies}, show_coded_ber: {show_coded_ber}, show_uncoded_ber: {show_uncoded_ber}"
            )
        self.caption = caption
        self.show_num_copies = show_num_copies
        self.show_coded_ber = show_coded_ber
        self.show_uncoded_ber = show_uncoded_ber
        self.show_bitrate = show_bitrate
        self.show_throughput = show_throughput
        self.show_per = show_per
        self.frame_per_sec = frame_per_sec
        self.logfreq = logfreq
        self.debug = debug
        gr.sync_block.__init__(self, name="err_stats",
                               in_sig=[np.uint8], out_sig=None)
        # Use QtWidgets.QWidget for widget initialization
        QtWidgets.QWidget.__init__(self)

        if dark:
            dark_palette = QtGui.QPalette()
            dark_palette.setColor(QtGui.QPalette.Window,
                                  QtGui.QColor(53, 53, 53))
            dark_palette.setColor(QtGui.QPalette.WindowText, Qt.white)
            dark_palette.setColor(QtGui.QPalette.Base,
                                  QtGui.QColor(25, 25, 25))
            dark_palette.setColor(
                QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
            dark_palette.setColor(QtGui.QPalette.ToolTipBase, Qt.white)
            dark_palette.setColor(QtGui.QPalette.ToolTipText, Qt.white)
            dark_palette.setColor(QtGui.QPalette.Text, Qt.white)
            dark_palette.setColor(QtGui.QPalette.Button,
                                  QtGui.QColor(53, 53, 53))
            dark_palette.setColor(QtGui.QPalette.ButtonText, Qt.white)
            dark_palette.setColor(QtGui.QPalette.BrightText, Qt.red)
            dark_palette.setColor(QtGui.QPalette.Link,
                                  QtGui.QColor(42, 130, 218))
            dark_palette.setColor(QtGui.QPalette.Highlight,
                                  QtGui.QColor(42, 130, 218))
            dark_palette.setColor(QtGui.QPalette.HighlightedText, Qt.black)
            self.setPalette(dark_palette)
            self.setAutoFillBackground(True)

        # define axis attributes as None
        self.bitrate_axisY = None
        self.throughput_axisY = None
        self.per_axisY = None
        self.num_copies_axisY = None
        self.coded_ber_axisY = None
        self.uncoded_ber_axisY = None

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        # Create QFrame container for all UI elements
        self.frame = QtWidgets.QFrame()
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setObjectName("MainFrame")
        self.frame.setStyleSheet("""
            #MainFrame {
                border: 2px solid gray;
                border-radius: 15px;
            }
        """)

        self.frameLayout = QtWidgets.QVBoxLayout()
        self.frame.setLayout(self.frameLayout)

        captionLayout = QtWidgets.QHBoxLayout()
        captionLayout.addStretch()
        # Create the checkbox, checked by default.
        self.showChartsCheckbox = QtWidgets.QCheckBox("")
        self.showChartsCheckbox.setChecked(True)
        self.showChartsCheckbox.stateChanged.connect(self.toggle_charts_layout)
        captionLayout.addWidget(self.showChartsCheckbox)

        # Create a horizontal layout for caption and checkbox
        self.caption_label = QtWidgets.QLabel(f"<b>{self.caption}</b>")
        self.caption_label.setAlignment(Qt.AlignCenter)
        self.caption_label.setStyleSheet("font-size: 17pt; color:red;")
        captionLayout.addWidget(self.caption_label)
        captionLayout.addStretch()
        
        # Add the caption layout to the frame layout instead of the main layout
        self.frameLayout.addLayout(captionLayout)

        # Create a container widget for charts layout so we can hide/show it easily.
        self.chartsWidget = QtWidgets.QWidget()
        self.chartsLayout = QtWidgets.QGridLayout()
        self.chartsWidget.setLayout(self.chartsLayout)
        # Add charts widget to the frame layout instead of the main layout
        self.frameLayout.addWidget(self.chartsWidget)

        # Number of seconds (data points) to display
        self.plot_secs = 10

        # -------------------------
        # Create QChart for Bitrate
        # -------------------------
        widget_id = 0
        if self.show_bitrate:
            self.bitrate_chart = QChart()
            self.bitrate_chart.setTitle("Bitrate (kbps)")
            self.bitrate_chart.legend().setVisible(False)  # disable legend
            self.bitrate_series = QLineSeries()
            self.bitrate_chart.addSeries(self.bitrate_series)
            self.bitrate_axisX = QValueAxis()
            self.bitrate_axisX.setRange(0, self.plot_secs - 1)
            self.bitrate_axisX.setLabelFormat("%d")
            self.bitrate_chart.addAxis(self.bitrate_axisX, Qt.AlignBottom)
            self.bitrate_series.attachAxis(self.bitrate_axisX)
            self.bitrate_axisY = QValueAxis()
            # initial range; will update later
            self.bitrate_axisY.setRange(0, 1)
            self.bitrate_chart.addAxis(self.bitrate_axisY, Qt.AlignLeft)
            self.bitrate_series.attachAxis(self.bitrate_axisY)
            self.bitrate_view = QChartView(self.bitrate_chart)
            self.chartsLayout.addWidget(
                self.bitrate_view, widget_id // cols, widget_id % cols)
            if dark:
                self.bitrate_chart.setTheme(QChart.ChartThemeDark)
            self.bitrate_chart.setTitleFont(
                QtGui.QFont("Arial", 14))  # set bigger title font
            widget_id += 1

        # ----------------------------
        # Create QChart for Throughput
        # ----------------------------
        if self.show_throughput:
            self.throughput_chart = QChart()
            self.throughput_chart.setTitle("Throughput (kbps)")
            self.throughput_chart.legend().setVisible(False)  # disable legend
            self.throughput_series = QLineSeries()
            self.throughput_chart.addSeries(self.throughput_series)
            self.throughput_axisX = QValueAxis()
            self.throughput_axisX.setRange(0, self.plot_secs - 1)
            self.throughput_axisX.setLabelFormat("%d")
            self.throughput_chart.addAxis(
                self.throughput_axisX, Qt.AlignBottom)
            self.throughput_series.attachAxis(self.throughput_axisX)
            self.throughput_axisY = QValueAxis()
            self.throughput_axisY.setRange(0, 1)
            self.throughput_chart.addAxis(self.throughput_axisY, Qt.AlignLeft)
            self.throughput_series.attachAxis(self.throughput_axisY)
            self.throughput_view = QChartView(self.throughput_chart)
            self.chartsLayout.addWidget(
                self.throughput_view, widget_id // cols, widget_id % cols)
            if dark:
                self.throughput_chart.setTheme(QChart.ChartThemeDark)
            self.throughput_chart.setTitleFont(
                QtGui.QFont("Arial", 14))  # set bigger title font
            widget_id += 1

        # ----------------------------------
        # Create QChart for Packet Error Rate
        # ----------------------------------
        if self.show_per:
            self.per_chart = QChart()
            self.per_chart.setTitle("Packet Error Rate")
            self.per_chart.legend().setVisible(False)  # disable legend
            self.per_series = QLineSeries()
            self.per_chart.addSeries(self.per_series)
            self.per_axisX = QValueAxis()
            self.per_axisX.setRange(0, self.plot_secs - 1)
            self.per_axisX.setLabelFormat("%d")
            self.per_chart.addAxis(self.per_axisX, Qt.AlignBottom)
            self.per_series.attachAxis(self.per_axisX)
            self.per_axisY = QValueAxis()
            self.per_axisY.setRange(0, 1)
            self.per_chart.addAxis(self.per_axisY, Qt.AlignLeft)
            self.per_series.attachAxis(self.per_axisY)
            self.per_view = QChartView(self.per_chart)
            self.chartsLayout.addWidget(
                self.per_view, widget_id // cols, widget_id % cols)
            if dark:
                self.per_chart.setTheme(QChart.ChartThemeDark)
            self.per_chart.setTitleFont(QtGui.QFont(
                "Arial", 14))  # set bigger title font
            widget_id += 1

        # ----------------------------------
        # Create QChart for Num Copies (if enabled)
        # ----------------------------------
        if self.show_num_copies:
            self.num_copies_chart = QChart()
            self.num_copies_chart.setTitle("Num Copies")
            self.num_copies_chart.legend().setVisible(False)
            self.num_copies_series = QLineSeries()
            self.num_copies_chart.addSeries(self.num_copies_series)
            self.num_copies_axisX = QValueAxis()
            self.num_copies_axisX.setRange(0, self.plot_secs - 1)
            self.num_copies_axisX.setLabelFormat("%d")
            self.num_copies_chart.addAxis(
                self.num_copies_axisX, Qt.AlignBottom)
            self.num_copies_series.attachAxis(self.num_copies_axisX)
            self.num_copies_axisY = QValueAxis()
            # changed range to 0-4 for integer ticks
            self.num_copies_axisY.setRange(0, 4)
            self.num_copies_axisY.setTickCount(5)  # force ticks at 0,1,2,3,4
            self.num_copies_axisY.setLabelFormat("%d")  # force integer labels
            self.num_copies_chart.addAxis(self.num_copies_axisY, Qt.AlignLeft)
            self.num_copies_series.attachAxis(self.num_copies_axisY)
            self.num_copies_view = QChartView(self.num_copies_chart)
            self.chartsLayout.addWidget(
                self.num_copies_view, widget_id // cols, widget_id % cols)
            self.NumCopies = [0]
            self.last_NumCopies = []
            if dark:
                self.num_copies_chart.setTheme(QChart.ChartThemeDark)
            self.num_copies_chart.setTitleFont(
                QtGui.QFont("Arial", 14))  # set bigger title font
            widget_id += 1

        # ----------------------------------
        # Create QChart for Coded BER (if enabled)
        # ----------------------------------
        if self.show_coded_ber:
            self.coded_ber_chart = QChart()
            self.coded_ber_chart.setTitle("Coded BER")
            self.coded_ber_chart.legend().setVisible(False)
            self.coded_ber_series = QLineSeries()
            self.coded_ber_chart.addSeries(self.coded_ber_series)
            self.coded_ber_axisX = QValueAxis()
            self.coded_ber_axisX.setRange(0, self.plot_secs - 1)
            self.coded_ber_axisX.setLabelFormat("%d")
            self.coded_ber_chart.addAxis(self.coded_ber_axisX, Qt.AlignBottom)
            self.coded_ber_series.attachAxis(self.coded_ber_axisX)
            self.coded_ber_axisY = QValueAxis()
            self.coded_ber_axisY.setRange(0, 1)
            self.coded_ber_chart.addAxis(self.coded_ber_axisY, Qt.AlignLeft)
            self.coded_ber_series.attachAxis(self.coded_ber_axisY)
            self.coded_ber_view = QChartView(self.coded_ber_chart)
            self.log_scale_checkbox_coded = QtWidgets.QCheckBox("log-scale", self.coded_ber_view)
            self.log_scale_checkbox_coded.stateChanged.connect(self.toggle_coded_ber_scale)
            self.log_scale_checkbox_coded.move(13, 13)
            if dark:
                self.log_scale_checkbox_coded.setStyleSheet("color: white;")
            self.chartsLayout.addWidget(
                self.coded_ber_view, widget_id // cols, widget_id % cols)
            self.last_CodedBERs = []
            self.CodedBERs = [EPS]
            if dark:
                self.coded_ber_chart.setTheme(QChart.ChartThemeDark)
            self.coded_ber_chart.setTitleFont(
                QtGui.QFont("Arial", 14))  # set bigger title font
            widget_id += 1

        # ----------------------------------
        # Create QChart for Uncoded BER (if enabled)
        # ----------------------------------
        if self.show_uncoded_ber:
            self.uncoded_ber_chart = QChart()
            self.uncoded_ber_chart.setTitle("Uncoded BER")
            self.uncoded_ber_chart.legend().setVisible(False)
            self.uncoded_ber_series = QLineSeries()
            self.uncoded_ber_chart.addSeries(self.uncoded_ber_series)
            self.uncoded_ber_axisX = QValueAxis()
            self.uncoded_ber_axisX.setRange(0, self.plot_secs - 1)
            self.uncoded_ber_axisX.setLabelFormat("%d")
            self.uncoded_ber_chart.addAxis(
                self.uncoded_ber_axisX, Qt.AlignBottom)
            self.uncoded_ber_series.attachAxis(self.uncoded_ber_axisX)
            self.uncoded_ber_axisY = QValueAxis()
            self.uncoded_ber_axisY.setRange(0, 1)
            self.uncoded_ber_chart.addAxis(
                self.uncoded_ber_axisY, Qt.AlignLeft)
            self.uncoded_ber_series.attachAxis(self.uncoded_ber_axisY)
            self.uncoded_ber_view = QChartView(self.uncoded_ber_chart)
            self.log_scale_checkbox_uncoded = QtWidgets.QCheckBox("log-scale", self.uncoded_ber_view)
            self.log_scale_checkbox_uncoded.stateChanged.connect(self.toggle_uncoded_ber_scale)
            self.log_scale_checkbox_uncoded.move(13, 13)
            if dark:
                self.log_scale_checkbox_uncoded.setStyleSheet("color: white;")
            self.chartsLayout.addWidget(
                self.uncoded_ber_view, widget_id // cols, widget_id % cols)
            self.last_UncodedBERs = []
            self.UncodedBERs = [EPS]
            if dark:
                self.uncoded_ber_chart.setTheme(QChart.ChartThemeDark)
            self.uncoded_ber_chart.setTitleFont(
                QtGui.QFont("Arial", 14))  # set bigger title font
            widget_id += 1
        # --------------------------------

        for r in range(self.chartsLayout.rowCount()):
            self.chartsLayout.setRowMinimumHeight(r, row_height)

        # -------------------------------
        # Add Stats Table
        # -------------------------------
        self.stats_table = QtWidgets.QTableWidget(0, 2)
        self.stats_table.setWordWrap(True)
        self.stats_table.horizontalHeader().setVisible(False)
        self.stats_table.verticalHeader().setVisible(False)
        # self.stats_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.stats_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Interactive)
        self.stats_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.stats_table.setColumnWidth(0, 200)  
        self.stats_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.chartsLayout.addWidget(self.stats_table, widget_id // cols, cols - 1)
        widget_id += 1

        self.last_seqno = -1
        self.last_dropped_seq = -1
        self.dropped_packets = 0
        self.correct_packets = 0
        self.total_packets = 0

        # -------------------------------
        # Initialize packet/KPI tracking
        # -------------------------------
        self.last_packet_remainder = 0
        self.first_packet = True
        self.packet_chunks = []
        self.packet_chunks_gt = []
        self.packets = []
        self.Bitrates = [0]
        self.last_Bitrates = []
        self.Throughputs = [0]
        self.last_Throughputs = []
        self.PERs = [0]
        self.last_PERs = []

        self.init_frame = True
        self.last_recvd_seqno = -1
        self.last_plot_tic = time.time()

        self.plot_cc = 0

        # At the end of __init__, add the QFrame to the main layout
        self.mainLayout.addWidget(self.frame)

        self.message_port_register_in(pmt.intern("reset"))
        self.set_msg_handler(pmt.intern("reset"), self.OnResetMsgRecvd)


        # Call update_plots() every 100 ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(int(self.logfreq / self.frame_per_sec * 1000))

        if self.show_coded_ber:
            self.log_scale_checkbox_coded.setChecked(True)
            self.toggle_coded_ber_scale(Qt.Checked)
        if self.show_uncoded_ber:
            self.log_scale_checkbox_uncoded.setChecked(True)
            self.toggle_uncoded_ber_scale(Qt.Checked)

    
    def OnResetMsgRecvd(self, msg):
        self.total_packets = 0
        self.correct_packets = 0
        self.dropped_packets = 0
        self.last_seqno = -1
        self.last_dropped_seq = -1
        self.update_plots()

    def toggle_charts_layout(self, state):
        """Toggle the visibility of the charts layout based on the checkbox state."""
        # When the checkbox is checked, show the charts; otherwise hide them.
        self.chartsWidget.setVisible(state == Qt.Checked)

    def toggle_coded_ber_scale(self, state):
        """Toggle log scale for the coded BER chart."""
        chart = self.coded_ber_chart
        series = self.coded_ber_series
        # Remove the existing y-axis.
        chart.removeAxis(self.coded_ber_axisY)
        if state == Qt.Checked:
            # Use logarithmic scale. Note: lower bound must be > 0.
            new_axis = QLogValueAxis()
            new_axis.setLabelFormat("%.1e")
            lower_bound = 1e-6
            max_val = max(self.CodedBERs) if self.CodedBERs and max(self.CodedBERs) > lower_bound else lower_bound * 10
            new_axis.setRange(lower_bound, max_val)
        else:
            new_axis = QValueAxis()
            new_axis.setLabelFormat("%.2f")
            max_val = max(self.CodedBERs) if self.CodedBERs else 1
            new_axis.setRange(0, max_val * 1.2 + 0.0001)
        chart.addAxis(new_axis, Qt.AlignLeft)
        series.attachAxis(new_axis)
        self.coded_ber_axisY = new_axis
        self.update_plots()

    def toggle_uncoded_ber_scale(self, state):
        """Toggle log scale for the uncoded BER chart."""
        chart = self.uncoded_ber_chart
        series = self.uncoded_ber_series
        chart.removeAxis(self.uncoded_ber_axisY)
        if state == Qt.Checked:
            new_axis = QLogValueAxis()
            new_axis.setLabelFormat("%.1e")
            lower_bound = 1e-6
            max_val = max(self.UncodedBERs) if self.UncodedBERs and max(self.UncodedBERs) > lower_bound else lower_bound * 10
            new_axis.setRange(lower_bound, max_val)
        else:
            new_axis = QValueAxis()
            new_axis.setLabelFormat("%.2f")
            max_val = max(self.UncodedBERs) if self.UncodedBERs else 1
            new_axis.setRange(0, max_val * 1.2 + 0.0001)
        chart.addAxis(new_axis, Qt.AlignLeft)
        series.attachAxis(new_axis)
        self.uncoded_ber_axisY = new_axis
        self.update_plots()

    def calculate_kpis(self):
        # Compute KPIs based on the collected data
        if len(self.last_PERs) >= self.frame_per_sec:
            self.last_Bitrates = [
                self.packets[-1]["packet_len"] * self.packets[-1]["rx_modtype"]
            ]
            if self.packets[-1]["rx_data_crc"]:
                self.last_PERs = [0]
                self.last_Throughputs = [
                    self.packets[-1]["packet_len"] *
                    self.packets[-1]["rx_modtype"]
                ]
            else:
                self.last_PERs = [1]
                self.last_Throughputs = [0]
            self.Bitrates.append(0)
            self.PERs.append(0)
            self.Throughputs.append(0)
        else:
            self.last_Bitrates.append(
                self.packets[-1]["packet_len"] * self.packets[-1]["rx_modtype"]
            )
            if self.packets[-1]["rx_data_crc"]:
                self.last_PERs.append(0)
                self.last_Throughputs.append(
                    self.packets[-1]["packet_len"] *
                    self.packets[-1]["rx_modtype"]
                )
            else:
                self.last_PERs.append(1)
                self.last_Throughputs.append(0)
        self.Bitrates[-1] = (
            sum(self.last_Bitrates)
            * (self.frame_per_sec / len(self.last_Bitrates))
            / 1000.0
        )
        self.Bitrates = self.Bitrates[-self.plot_secs:]
        self.PERs[-1] = sum(self.last_PERs) / len(self.last_PERs)
        self.PERs = self.PERs[-self.plot_secs:]
        self.Throughputs[-1] = (
            sum(self.last_Throughputs)
            * (self.frame_per_sec / len(self.last_Throughputs))
            / 1000.0
        )
        self.Throughputs = self.Throughputs[-self.plot_secs:]
        if self.show_num_copies:
            if len(self.last_NumCopies) >= self.frame_per_sec:
                self.last_NumCopies = [
                    self.packets[-1].get("rx_num_copies", 0)]
                self.NumCopies.append(0)
            else:
                self.last_NumCopies.append(
                    self.packets[-1].get("rx_num_copies", 0))
            self.NumCopies[-1] = sum(self.last_NumCopies) / \
                len(self.last_NumCopies)
            self.NumCopies = self.NumCopies[-self.plot_secs:]
        if self.show_coded_ber:
            if len(self.last_CodedBERs) >= self.frame_per_sec:
                self.last_CodedBERs = [self.packets[-1].get("rx_coded_ber", 0)]
                self.CodedBERs.append(0)
            else:
                self.last_CodedBERs.append(self.packets[-1].get("rx_coded_ber", 0))
            self.CodedBERs[-1] = sum(self.last_CodedBERs) / len(self.last_CodedBERs) + EPS
            self.CodedBERs = self.CodedBERs[-self.plot_secs:]
        if self.show_uncoded_ber:
            if len(self.last_UncodedBERs) >= self.frame_per_sec:
                self.last_UncodedBERs = [self.packets[-1].get("rx_uncoded_ber", 0)]
                self.UncodedBERs.append(0)
            else:
                self.last_UncodedBERs.append(self.packets[-1].get("rx_uncoded_ber", 0))
            self.UncodedBERs[-1] = sum(self.last_UncodedBERs) / len(self.last_UncodedBERs) + EPS
            self.UncodedBERs = self.UncodedBERs[-self.plot_secs:]

    def update_chart(self, series, axisY, data):
        # Clear the current series and update with new points.
        series.clear()
        # Calculate an x-offset if data length is less than plot_secs.
        offset = self.plot_secs - len(data)
        for i, value in enumerate(data):
            series.append(offset + i, value)
        # Update y-axis range based on the data (mimicking the original scaling)
        max_val = max(data) if data else 1
        if self.show_num_copies:
            if axisY == self.num_copies_axisY:
                max_num_copies_int = int(max_val + 1)
                axisY.setRange(0, max_num_copies_int)
                axisY.setTickCount(max_num_copies_int + 1)
                return
        if len(data) > 2:
            max_val = max(data[:-1])
        # Check if the axis is logarithmic and adjust accordingly.
        if axisY.__class__.__name__ == "QLogValueAxis":
            lower_bound = EPS
            if len(data) > 2:
                lower_bound = min(data[:-1]) / 9.0
            axisY.setRange(lower_bound, (max_val+EPS) * 9.0)
            axisY.setMinorTickCount(-1)
            axisY.setLabelFormat("%.1e")
        else:
            if axisY in [self.bitrate_axisY, self.throughput_axisY]:
                max_y = max_val * 1.2 + 0.5
            else:
                max_y = max_val * 1.2 + 0.001
            if axisY in [self.coded_ber_axisY, self.uncoded_ber_axisY]:
                axisY.setTickCount(5)
                # If the BER is very small, switch to exponential notation
                if max_y < 1e-5:
                    axisY.setLabelFormat("%.2e")
                else:
                    axisY.setLabelFormat("%.4f")
            axisY.setRange(0, max_y)

    def update_plots(self):
        # Update all the charts with newly computed KPI data.
        if self.plot_cc % self.logfreq == 0:
            if self.show_bitrate:
                self.update_chart(
                    self.bitrate_series, self.bitrate_axisY, self.Bitrates
                )
            if self.show_throughput:
                self.update_chart(
                    self.throughput_series, self.throughput_axisY, self.Throughputs
                )
            if self.show_per:
                self.update_chart(self.per_series, self.per_axisY, self.PERs)
            if self.show_num_copies:
                self.update_chart(
                    self.num_copies_series, self.num_copies_axisY, self.NumCopies
                )
            if self.show_coded_ber:
                self.update_chart(
                    self.coded_ber_series, self.coded_ber_axisY, self.CodedBERs
                )
            if self.show_uncoded_ber:
                self.update_chart(
                    self.uncoded_ber_series, self.uncoded_ber_axisY, self.UncodedBERs
                )
        if self.total_packets:
            stats = [
                ("Total Received Packets", str(self.total_packets)),
                ("Correct Received Packets",
                 f"{self.correct_packets} ({self.correct_packets/self.total_packets*100:.2f}%)"),
                ("Dropped Packets",
                 f"{self.dropped_packets} ({self.dropped_packets/(self.dropped_packets+self.total_packets)*100:.2f}%)"),
                ("Last Received Seq#", str(self.last_seqno)),
                ("Last Dropped Seq#", str(self.last_dropped_seq)),
                ("Packet Error Rate", f"{self.PERs[-1]*100:.2f}%"),
                ("Throughput", f"{self.Throughputs[-1]:.2f} kbps")
            ]
            if self.show_coded_ber:
                stats.append(("Coded BER", f"{(self.CodedBERs[-1]-EPS):.2e}"))
            if self.show_uncoded_ber:
                stats.append(("Uncoded BER", f"{(self.UncodedBERs[-1]-EPS):.2e}"))
            if self.show_num_copies:
                stats.append(("Num Copies", str(int(math.ceil(self.NumCopies[-1])))))
            if 'hll_snr_rbs_db' in self.tags:
                dict_val = self.tags['hll_snr_rbs_db']['value']
                str_val = ''
                for k in sorted(dict_val.keys()):
                    str_val += f'C{k}: (' + ', '.join([f'{v:.2f}' for v in dict_val[k]]) + ')\n'
                str_val = str_val.strip()
                stats.append(("HLL RB SNRs (dB)", str_val))
            if 'snr_rbs_db' in self.tags:
                str_val = '(' + ', '.join([f'{v:.2f}' for v in self.tags['snr_rbs_db']['value']]) + ')'
                stats.append(("RB SNRs (dB)", str_val))


            self.stats_table.setRowCount(len(stats))
            for row, (label, value) in enumerate(stats):
                self.stats_table.setItem(row, 0, QtWidgets.QTableWidgetItem(label))
                self.stats_table.setItem(row, 1, QtWidgets.QTableWidgetItem(value))
        self.plot_cc += 1
        # handle UI events
        QtWidgets.QApplication.processEvents()
        # Force a refresh of the charts
        if self.show_bitrate:
            self.bitrate_chart.update()
        if self.show_throughput:
            self.throughput_chart.update()
        if self.show_per:
            self.per_chart.update()
        if self.show_coded_ber:
            self.coded_ber_chart.update()
        if self.show_uncoded_ber:
            self.uncoded_ber_chart.update()

    def group_reverse(self, lst, n=5):
        return [lst[i: i + n] for i in range(len(lst) - 1, -1, -n)][::-1]

    def forecast(self, noutput_items, ninputs):
        return [noutput_items] * ninputs

    def work(self, input_items, output_items):
        ninput_items = min([len(items) for items in input_items])
        if self.debug:
            print("***" * 10)
            print(
                f"[err_stats]  len(input_items): ",
                [len(items) for items in input_items],
            )
        if self.last_packet_remainder == 0:
            self.tags = {
                pmt.to_python(tag.key): {
                    "offset": tag.offset,
                    "value": pmt.to_python(tag.value),
                }
                for tag in self.get_tags_in_window(0, 0, 1)
            }
            if self.debug:
                print(self.tags)

            assert "packet_len" in self.tags, "packet_len tag not found!"
            self.last_packet_remainder = self.tags["packet_len"]["value"]
            self.last_seqno = self.tags["rx_seqno"]["value"]
            self.total_packets += 1
            if self.tags["rx_data_crc"]["value"] and self.tags["rx_ctrl_ok"]["value"]:
                self.correct_packets += 1

        if self.init_frame:
            self.init_frame = False
        else:
            if self.last_recvd_seqno + 1 != self.tags["rx_seqno"]["value"]:
                for dropped_packet_seqno in range(self.last_recvd_seqno + 1, self.tags["rx_seqno"]["value"]):
                    # print(f"[err_stats] {self.caption} Dropped Packet: {dropped_packet_seqno}")
                    self.last_dropped_seq = dropped_packet_seqno
                    self.dropped_packets += 1
                    self.packets.append({})
                    self.packets[-1] = self.packets[-2].copy()
                    self.packets[-1]["rx_seqno"] = dropped_packet_seqno
                    self.packets[-1]["rx_data_crc"] = False
                    self.packets[-1]["rx_ctrl_ok"] = False
        self.last_recvd_seqno = self.tags["rx_seqno"]["value"]
        to_consume = min(ninput_items, self.last_packet_remainder)
        self.packet_chunks.append(input_items[0][:to_consume])
        self.last_packet_remainder -= to_consume
        if self.last_packet_remainder == 0:
            self.packets.append({})
            packet = np.concatenate(self.packet_chunks)
            for t in self.tags:
                self.packets[-1][t] = self.tags[t]["value"]
            self.calculate_kpis()
            self.packet_chunks = []
            self.packet_chunks_gt = []
        return to_consume
