% Generate 4Tx-2Ss SU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% data output folder
datadir = '../data/su_mimo_he/';

% use 582 for QPSK, 1168 for 16QAM, 1752 for 64QAM, 2338 for 256QAM
% to generate 10 OFDM symbols per frame

modtype = "QPSK";
psdulen = 582;
su_mimo_sigen_he_csd("4t2s_csd", psdulen, modtype, 'LDPC', datadir);

modtype = "16QAM";
psdulen = 1168;
su_mimo_sigen_he_csd("4t2s_csd", psdulen, modtype, 'LDPC', datadir);

modtype = "64QAM";
psdulen = 1752;
su_mimo_sigen_he_csd("4t2s_csd", psdulen, modtype, 'LDPC', datadir);

modtype = "256QAM";
psdulen = 2338;
su_mimo_sigen_he_csd("4t2s_csd", psdulen, modtype, 'LDPC', datadir);

