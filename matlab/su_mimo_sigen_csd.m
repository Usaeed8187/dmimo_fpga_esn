function su_mimo_sigen_csd(mimotype, psdulen, modtype, cctype, datadir)
% Generate Tx signal for 4x2 MIMO downlink
% using CSD for 2-stream transmission,
% with additional 4x4 HT-LTF for CSI feedback

% System configuration
cfg = sys_config(mimotype, psdulen, modtype, cctype);

% Create data output folder
[~, ~, ~] = mkdir(fullfile(datadir, mimotype, modtype)); % create output folder

% Load 4x4 HT LTF signals (for CSI feedback)
b = load('beacon4x4.mat');
load('lltfx4.mat','lltfx4');

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
txPSDU = randi([0 1], psdulen*8, 1, 'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% Time-domain preamble signals (HT-SIG, HT-STF, HT-LTF)
% preamble = sqrt(2)*[b.lstf; b.lltf; b.lsig; b.htsig; b.htstf; b.htltf; htltfx];
preamble = sqrt(2)*[b.lstf; lltfx4; b.lsig; b.htsig; b.htstf; b.htltf; htltfx];

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
for k=1:cfg.Nt
    write_complex_binary(txsig(:,k), ...
        sprintf('%s/%s/%s/txsig_s%d.bin',datadir,mimotype,modtype,k));
end
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

for k=1:cfg.Nss
    write_complex_binary(txdsyms(:,:,k), ...
        sprintf("%s/%s/%s/txdsyms_s%d.bin",datadir,mimotype,modtype,k));
end

fprintf("\n")

