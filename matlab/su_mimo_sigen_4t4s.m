% Generate 4Tx-4Ss SU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/su_mimo/';

% use 1038 K, 2078 for 16QAM, 3118 for 64QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 1038;
su_mimo_sigen("4t4s", psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 2078;
su_mimo_sigen("4t4s", psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 3118;
su_mimo_sigen("4t4s", psdulen, modtype, 'LDPC', datadir);

