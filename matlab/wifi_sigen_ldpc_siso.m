clear
addpath("./wlan", "./gnuradio/");
save_txsig = true;

% System configuration
mimotype = '1x1';
modtype = "64QAM"; % QPSK/16QAM/64QAM
% use 250 for QPSK, 510 for 16QAM, 778 for 64QAM, 1038/1298 for 256QAM
PSDULength = 778; % 40/50 OFDM symbols
cfg = sys_config(mimotype, PSDULength, modtype, 'LDPC');

% Load 802.11 beacon signals
load('beacon2x2.mat')

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1],PSDULength*8,1,'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);
txdata = 1/sqrt(2) * txdata;

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF, HT-DATA)
preamble = [lstf(:,1); lltf(:,1); lsig(:,1); ...
           htsig(:,1); htstf(:,1); htltf(1:80,1);];

% Prepare transmitter signal for USRP
txFrame = reshape([preamble; txdata], (cfg.Nfft+cfg.Ncp), [], cfg.Nt);
txFrame = reshape(txFrame, [], cfg.Nt);

% Convert Tx signals to range (-1,1)
% scaling = 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
scaling = 0.5;
txsig = scaling * txFrame;

% Zero-padding for burst transmission
padding = zeros(1000, cfg.Nt);
txsig = [padding; txsig; padding];

% Show system parameters
nLegacyPreamble = 5;
nHTPreamble = (size(preamble,1) - 80*nLegacyPreamble) / (cfg.Nfft+cfg.Ncp);
nDataSymbols = size(txdata, 1) / (cfg.Nfft+cfg.Ncp);
fprintf("Number of HT symbols: %d\nNumber of Data symbols: %d\n", ...
        nHTPreamble, nDataSymbols);
fprintf("Tx signal frame size: %d\n", size(txsig,1));
fprintf("Tx frame data length (bits): %d\n", size(strmdata,1));

% Save data fro GNU Radio implementation
[st, ~, ~] = mkdir(fullfile("data", mimotype, modtype));  % create output folder
if exist('save_txsig', 'var') && save_txsig
    fprintf("Saving GNU Radio Tx data files ...\n")
    write_complex_binary(txsig(:,1), ...
        sprintf('./data/%s/%s/txsig_1x1_s1.bin',mimotype,modtype));
    write_complex_binary(scaling*preamble, ...
        sprintf('./data/%s/ncjt_preamble_1x1.bin',mimotype));
end

fid = fopen(sprintf('./data/%s/%s/enc_data_1x1.bin',mimotype,modtype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('./data/%s/%s/strm_data_1x1.bin',mimotype,modtype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_tmp = reshape(permute(txdsyms, [3, 1, 2]), [], 1);
write_complex_binary(txdsyms_tmp, ...
    sprintf("./data/%s/%s/txdsyms_1x1.bin",mimotype,modtype));


