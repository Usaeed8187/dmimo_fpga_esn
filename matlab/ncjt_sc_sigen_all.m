% Generate all NCJT single cluster Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/sc_ncjt/';

% use 250 for QPSK, 510 for 16QAM, 778 for 64QAM, 1038 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 250;
ncjt_sc_sigen(psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 510;
ncjt_sc_sigen(psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 778;
ncjt_sc_sigen(psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 1038;
ncjt_sc_sigen(psdulen, modtype, 'LDPC', datadir);

