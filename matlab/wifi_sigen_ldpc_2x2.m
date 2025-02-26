clear
addpath("./wlan", "./gnuradio/");
save_txsig = true;

% System configuration
mimotype = '2x2';
modtype = "256QAM"; % QPSK/16QAM/64QAM/256QAM
% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
PSDULength = 2078;  % for 40 OFDM symbols
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

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF, HT-DATA)
preamble = [lstf; lltf; lsig; htsig; htstf; htltf];  % standard one

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
fprintf("Tx signal scaling: %.15f\n", scaling);

% Save data fro GNU Radio implementation
[st, ~, ~] = mkdir(fullfile("data", mimotype, modtype));  % create output folder
if exist('save_txsig', 'var') && save_txsig
    fprintf("Saving GNU Radio Tx data files ...\n")
    write_complex_binary(txsig(:,1), ...
        sprintf('./data/%s/%s/txsig_2x2_s1.bin',mimotype,modtype));
    write_complex_binary(txsig(:,2), ...
        sprintf('./data/%s/%s/txsig_2x2_s2.bin',mimotype,modtype));
    write_complex_binary(scaling*preamble, ...
        sprintf('./data/%s/%s/preamble_2x2.bin',mimotype,modtype));
end

fid = fopen(sprintf('./data/%s/%s/enc_data_2x2.bin',mimotype,modtype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('./data/%s/%s/strm_data_2x2.bin',mimotype,modtype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

fid = fopen(sprintf('./data/%s/%s/tx_strm_data_2x2.bin',mimotype,modtype),'wb');
txstrmdata = strmdata.';
fwrite(fid, txstrmdata, "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_tmp = reshape(permute(txdsyms, [3, 1, 2]), [], 1); % (Nss, Nsc, Nsyms)
write_complex_binary(txdsyms_tmp, ...
    sprintf("./data/%s/%s/txdsyms_2x2.bin",mimotype,modtype));
write_complex_binary(txdsyms(:,:,1), ...
    sprintf("./data/%s/%s/txdsyms_2x2_s1.bin",mimotype,modtype));
write_complex_binary(txdsyms(:,:,2), ...
    sprintf("./data/%s/%s/txdsyms_2x2_s2.bin",mimotype,modtype));


