% Generate 2Tx-2Ss SU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/su_mimo_he/';

% use 816 for QPSK, 1636 for 16QAM, 2452 for 64QAM, 3272 for 256QAM
% to generate 14 OFDM symbols per frame

modtype = "QPSK";
psdulen = 816;
su_mimo_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 1636;
su_mimo_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 2452;
su_mimo_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 3272;
su_mimo_sigen_he("2t2s", psdulen, modtype, 'LDPC', datadir);

