% Generate 4Tx-2Ss MU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System params
mimotype = "4t2s_csd";
cctype = 'LDPC';

% data output folder
datadir = '../data/mu_mimo/';

% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 510;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 1038;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 1558;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 2078;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_csd(cfg, mimotype, psdulen, modtype, datadir);
