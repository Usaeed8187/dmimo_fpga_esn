% Generate 4Tx-2Ss SU-MIMO Tx signal for testing

clear
addpath("./wlan", "./gnuradio/");

% System configuration
mimotype = '4t2s_csd';
cctype = 'LDPC';

% data output folder
datadir = '../data/su_mimo_nr/';

% use 582 for QPSK, 1168 for 16QAM, 1752 for 64QAM, 2338 for 256QAM
% to generate 10 OFDM symbols per frame

modtype = "QPSK";
psdulen = 460;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
su_mimo_sigen_he_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "16QAM";
psdulen = 920;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
su_mimo_sigen_he_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "64QAM";
psdulen = 1382;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
su_mimo_sigen_he_csd(cfg, mimotype, psdulen, modtype, datadir);

modtype = "256QAM";
psdulen = 1846;
cfg = sys_config_nr(mimotype, psdulen, modtype, cctype);
su_mimo_sigen_he_csd(cfg, mimotype, psdulen, modtype, datadir);

