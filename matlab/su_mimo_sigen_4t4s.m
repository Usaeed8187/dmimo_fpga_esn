% Generate 4Tx-4Ss SU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System params
mimotype = "4t4s";
cctype = 'LDPC';

% data output folder
datadir = '../data/su_mimo/';

% use 1038 K, 2078 for 16QAM, 3118 for 64QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 1038;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
su_mimo_sigen(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 2078;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
su_mimo_sigen(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 3118;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
su_mimo_sigen(cfg, mimotype, psdulen, modtype, datadir);

