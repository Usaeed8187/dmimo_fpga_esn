% Generate 2Tx-2Ss MU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System params
mimotype = "2t2s";
cctype = 'LDPC';

% data output folder
datadir = '../data/mu_mimo_nr/';

% use 460 for QPSK, 920 for 16QAM, 1382 for 64QAM, 1846 for 256QAM
% to generate 14 OFDM symbols per frame

modtype = "QPSK";
psdulen = 460;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 920;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 1382;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 1846;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
mu_mimo_ul_sigen_he(cfg, mimotype, psdulen, modtype, datadir);
