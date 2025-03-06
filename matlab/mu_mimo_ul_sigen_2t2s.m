% Generate 2Tx-2Ss MU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/mu_mimo/';

% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 510;
mu_mimo_ul_sigen("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 1038;
mu_mimo_ul_sigen("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 1558;
mu_mimo_ul_sigen("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 2078;
mu_mimo_ul_sigen("2t2s", psdulen, modtype, 'LDPC', datadir);
