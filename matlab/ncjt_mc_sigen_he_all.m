% Generate all NCJT single cluster Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System configuration
mimotype = '2t2s';
cctype = 'LDPC';

% data output folder
datadir = '../data/mc_ncjt_he/';

% use 816 for QPSK, 1636 for 16QAM, 2452 for 64QAM, 3272 for 256QAM
% to generate 40 OFDM symbols per frame

modtype = "QPSK";
psdulen = 816;
cfg = sys_config_he(mimotype, psdulen, modtype, cctype);
ncjt_mc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 1636;
cfg = sys_config_he(mimotype, psdulen, modtype, cctype);
ncjt_mc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 2452;
cfg = sys_config_he(mimotype, psdulen, modtype, cctype);
ncjt_mc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 3272;
cfg = sys_config_he(mimotype, psdulen, modtype, cctype);
ncjt_mc_sigen_he(cfg, mimotype, psdulen, modtype, datadir);

