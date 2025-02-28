clear
addpath("./wlan", "./gnuradio/");
save_txsig = true;

% System configuration
mimotype = '2t1s_svd';
modtype = "16QAM"; % QPSK/16QAM/64QAM/256QAM
% use 510 for QPSK, 1038 for 16QAM, 1558 for 64QAM, 2078 for 256QAM
PSDULength = 510;  % for 40 OFDM symbols
cfg = sys_config(mimotype, PSDULength, modtype, 'LDPC');

% Load 802.11 beacon signals
load('beacon2x2.mat')

if strcmpi(mimotype, '2t1s_svd')
    cfg.wideband = true;
    cfg.csi_dir = '/tmp/chanest_s1.bin';
    ltf_precoded_time_domain = cal_ltf_precoded_time_domain(cfg);
    htltfx = ltf_precoded_time_domain;
    % htltfx = cat(1, [htltfx(:,1), htltfx(:, 2)], [htltfx(:,1), htltfx(:, 2)]);
else
    hold = 1;
end
% orthonal LTF for 2 Tx
% htltfx = cat(1, ltf_precoded_time_domain, ltf_precoded_time_domain);
% htltfx = cat(1, [htltf(1:80,1), zeros(80, 1)], ...
%                 [zeros(80, 1), htltf(1:80,1)]);

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1],PSDULength*8,1,'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF, HT-DATA)
preamble = sqrt(2)* [lstf; lltf; lsig; htsig; htstf; htltfx];

% Prepare transmitter signal for USRP
txFrame = reshape([preamble; txdata], (cfg.Nfft+cfg.Ncp), [], cfg.Nt);
txFrame = reshape(txFrame, [], cfg.Nt);

% Convert Tx signals to range (-1,1)
scaling = 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
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

% Save data for GNU Radio implementation
[st, ~, ~] = mkdir(fullfile("data", mimotype, modtype));  % create output folder
if exist('save_txsig', 'var') && save_txsig
    fprintf("Saving GNU Radio Tx data files ...\n")
    write_complex_binary(txsig(:,1), ...
        sprintf('./data/%s/%s/txsig_%s_s1.bin',mimotype,modtype,mimotype));
    write_complex_binary(txsig(:,2), ...
        sprintf('./data/%s/%s/txsig_%s_s2.bin',mimotype,modtype,mimotype));
    write_complex_binary(scaling*preamble, ...
        sprintf('./data/%s/preamble_%s.bin',mimotype,mimotype));
end

fid = fopen(sprintf('./data/%s/%s/enc_data_%s.bin',mimotype,modtype,mimotype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('./data/%s/%s/strm_data_%s.bin',mimotype,modtype,mimotype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_tmp = reshape(permute(txdsyms, [3, 1, 2]), [], 1);
write_complex_binary(txdsyms_tmp, ...
    sprintf("./data/%s/%s/txdsyms_%s.bin",mimotype,modtype,mimotype));


