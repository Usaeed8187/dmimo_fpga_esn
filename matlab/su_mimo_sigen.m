function su_mimo_sigen(mimotype, psdulen, modtype, cctype, datadir)

% System configuration
cfg = sys_config(mimotype, psdulen, modtype, cctype);

% Create data output folder
[~, ~, ~] = mkdir(fullfile(datadir, mimotype, modtype)); % create output folder

% Load 802.11 beacon signals
b = load('beacon2x2.mat');

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1], psdulen*8, 1, 'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF)
preamble = [b.lstf; b.lltf; b.lsig; b.htsig; b.htstf; b.htltf];

% Prepare transmitter signal for USRP
txFrame = reshape([preamble; txdata], (cfg.Nfft+cfg.Ncp), [], cfg.Nt);
txFrame = reshape(txFrame, [], cfg.Nt);

% Convert Tx signals to range (-1,1)
% scaling = 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
scaling = 0.5; % fixed scaling
txsig = signal_clipping(scaling * txFrame);

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
fprintf("Tx signal scaling: %.15f\n", scaling);

% Save data fro GNU Radio implementation
fprintf("Saving GNU Radio Tx data files ...\n")
write_complex_binary(txsig(:,1), ...
    sprintf('%s/%s/%s/txsig_s1.bin',datadir,mimotype,modtype));
write_complex_binary(txsig(:,2), ...
    sprintf('%s/%s/%s/txsig_s2.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*preamble, ...
    sprintf('%s/%s/%s/preamble.bin',datadir,mimotype,modtype));

fid = fopen(sprintf('%s/%s/%s/enc_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/strm_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/tx_strm_data.bin',datadir,mimotype,modtype),'wb');
txstrmdata = strmdata.';
fwrite(fid, txstrmdata, "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_intlv = reshape(permute(txdsyms, [3, 1, 2]), [], 1); % (Nss, Nsc, Nsyms)
write_complex_binary(txdsyms_intlv, ...
    sprintf("%s/%s/%s/txdsyms.bin",datadir,mimotype,modtype));
write_complex_binary(txdsyms(:,:,1), ...
    sprintf("%s/%s/%s/txdsyms_s1.bin",datadir,mimotype,modtype));
write_complex_binary(txdsyms(:,:,2), ...
    sprintf("%s/%s/%s/txdsyms_s2.bin",datadir,mimotype,modtype));

fprintf("\n")

