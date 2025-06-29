% Generate all NCJT single cluster Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System configuration
mimotype = '1t1s';
cctype = 'LDPC';

% data output folder
datadir = '../data/sc_ncjt/';

% use 250 for QPSK, 510 for 16QAM, 778 for 64QAM, 1038 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 250;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 510;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 778;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 1038;
cfg = sys_config(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen(cfg, mimotype, psdulen, modtype, datadir);

