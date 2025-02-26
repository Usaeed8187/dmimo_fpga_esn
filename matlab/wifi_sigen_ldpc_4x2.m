% Generate Tx signal for 4x2 MIMO downlink
% using CSD for 2-stream transmission,
% with additional 4x4 HT-LTF for CSI feedback
clear
addpath("./wlan", "./gnuradio/");
save_txsig = true;

% System configuration
mimotype = '4x2csd';
modtype = "16QAM"; % QPSK/16QAM/64QAM
% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
PSDULength = 1038;  % for 40 OFDM symbols
cfg = sys_config(mimotype, PSDULength, modtype, 'LDPC');

% Load 4x4 HT LTF signals (for CSI feedback)
load('beacon4x4.mat');

% Spatial mapping for HT-LTF (for data reception)
ltf = load('beacon2x2.mat', 'htltf').htltf;
ltf = reshape(ltf, 80, [], 2);
ltfx = fftshift(fft(ltf(17:end,:,:)), 1);
htltfx = ifft(ifftshift(spatialMapping(ltfx, cfg), 1));
htltfx = sqrt(0.5)*reshape([htltfx(49:end,:,:); htltfx], [], 4); % add CP

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1],PSDULength*8,1,'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF, HT-DATA)
preamble = sqrt(2)*[lstf; lltf; lsig; htsig; htstf; htltf; htltfx];

% Prepare transmitter signal for USRP
txFrame = reshape([preamble; txdata], (cfg.Nfft+cfg.Ncp), [], cfg.Nt);
txFrame = reshape(txFrame, [], cfg.Nt);

% Convert Tx signals to range (-1,1)
scaling = 0.5; % 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
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

% Save data fro GNU Radio implementation
[st, ~, ~] = mkdir(fullfile("data", mimotype, modtype));  % create output folder
if exist('save_txsig', 'var') && save_txsig
    fprintf("Saving GNU Radio Tx data files ...\n")
    write_complex_binary(txsig(:,1), ...
        sprintf('./data/%s/%s/txsig_4x2_s1.bin',mimotype,modtype));
    write_complex_binary(txsig(:,2), ...
        sprintf('./data/%s/%s/txsig_4x2_s2.bin',mimotype,modtype));
    write_complex_binary(txsig(:,3), ...
        sprintf('./data/%s/%s/txsig_4x2_s3.bin',mimotype,modtype));
    write_complex_binary(txsig(:,4), ...
        sprintf('./data/%s/%s/txsig_4x2_s4.bin',mimotype,modtype));
    write_complex_binary(scaling*preamble, ...
        sprintf('./data/%s/preamble_4x2.bin',mimotype));
end

fid = fopen(sprintf('./data/%s/%s/enc_data_4x2.bin',mimotype,modtype), 'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('./data/%s/%s/strm_data_4x2.bin',mimotype,modtype), 'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_tmp = reshape(permute(txdsyms, [3, 1, 2]), [], 1);
write_complex_binary(txdsyms_tmp, ...
    sprintf("./data/%s/%s/txdsyms_4x2.bin", mimotype, modtype));


