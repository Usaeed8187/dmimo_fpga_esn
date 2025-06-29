% Generate all NCJT single cluster Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System configuration
mimotype = '1t1s';
cctype = 'LDPC';

% data output folder
datadir = '../data/sc_ncjt_nr/';

% use 228 for QPSK, 460 for 16QAM, 690 for 64QAM, 922 for 256QAM
% to generate 14 OFDM symbols per frame

modtype = "QPSK";
psdulen = 228;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 460;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 690;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 922;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
ncjt_sc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

