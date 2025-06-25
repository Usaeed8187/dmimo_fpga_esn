% Generate all NCJT single cluster Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/sc_ncjt_he/';

% use 406 for QPSK, 816 for 16QAM, 1226 for 64QAM, 1532 for 256QAM
% to generate 14 OFDM symbols per frame

modtype = "QPSK";
psdulen = 406;
ncjt_sc_sigen_he(psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 816;
ncjt_sc_sigen_he(psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 1226;
ncjt_sc_sigen_he(psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 1632;
ncjt_sc_sigen_he(psdulen, modtype, 'LDPC', datadir);

