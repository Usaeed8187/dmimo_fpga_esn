% Generate 2Tx-2Ss MU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/mu_mimo_he/';

% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 816;
mu_mimo_ul_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 1636;
mu_mimo_ul_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 2452;
mu_mimo_ul_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 3272;
mu_mimo_ul_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);
