function ncjt_mc_sigen_he(cfg, mimotype, psdulen, modtype, datadir)

% Create data output folder
[~, ~, ~] = mkdir(fullfile(datadir, mimotype, modtype)); % create output folder

% Load 802.11 beacon signals
b = load('beacon2x2he.mat');
bx = load('heltfx.mat');

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1], psdulen*8, 1, 'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% Dummy STBC transmitter
txdata1 = txdata;
txdata2 = txdata;

% scaling for Nsc=140
ltf_scaling = sqrt(242/cfg.Nsc);
bx.heltfx = ltf_scaling * bx.heltfx;

% Time-domain preamble signals (HE-SIG, HE-STF, HE-LTF)
heltfx1 = cat(1, bx.heltfx, zeros(size(bx.heltfx)));
heltfx2 = cat(1, zeros(size(bx.heltfx)), bx.heltfx);
preamble1 = [b.lstf; b.lltf; b.lsig; b.hesiga; b.hestf; b.hestf; heltfx1];
preamble2 = [b.lstf; b.lltf; b.lsig; b.hesiga; b.hestf; b.hestf; heltfx2];

% Prepare transmitter signal for USRP
txFrame1 = reshape([preamble1; txdata1], [], cfg.Nt);
txFrame2 = reshape([preamble2; txdata2], [], cfg.Nt);

% Convert Tx signals to range (-1,1)
% scaling = 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
scaling = 0.5; % fixed scaling
txsig1 = signal_clipping(scaling * txFrame1);
txsig2 = signal_clipping(scaling * txFrame2);

% Zero-padding for burst transmission
padding = zeros(1000, 2);
txsig1 = [padding; txsig1; padding];
txsig2 = [padding; txsig2; padding];

% Show system parameters
nLegacyPreamble = 5;
nHTPreamble = (size(preamble1,1) - 80*nLegacyPreamble) / (cfg.Nfft+cfg.Ncp);
nDataSymbols = size(txdata, 1) / (cfg.Nfft+cfg.Ncp);
fprintf("Number of HT symbols: %d\nNumber of Data symbols: %d\n", ...
        nHTPreamble, nDataSymbols);
fprintf("Tx signal frame size: %d\n", size(txsig1,1));
fprintf("Tx frame data length (bits): %d\n", size(strmdata,1));
fprintf("Tx signal scaling: %.15f\n", scaling);

% Save data fro GNU Radio implementation
fprintf("Saving GNU Radio Tx data files ...\n")
write_complex_binary(txsig1(:,1), ...
    sprintf('%s/%s/%s/txsig_ue1_s1.bin',datadir,mimotype,modtype));
write_complex_binary(txsig1(:,2), ...
    sprintf('%s/%s/%s/txsig_ue1_s2.bin',datadir,mimotype,modtype));
write_complex_binary(txsig2(:,1), ...
    sprintf('%s/%s/%s/txsig_ue2_s1.bin',datadir,mimotype,modtype));
write_complex_binary(txsig2(:,2), ...
    sprintf('%s/%s/%s/txsig_ue2_s2.bin',datadir,mimotype,modtype));

% Save preamble signals
write_complex_binary(scaling*preamble1, ...
    sprintf('%s/%s/%s/ncjt_preamble_ue1.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*preamble2, ...
    sprintf('%s/%s/%s/ncjt_preamble_ue2.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*[preamble1, preamble2], ...
    sprintf('%s/%s/%s/ncjt_preamble.bin',datadir,mimotype,modtype));

% Save preamble with v2 lltf
lx = load('lltfx2.mat','lltfx');
v2preamble1 = [b.lstf; lx.lltfx; b.lsig; b.hesiga; b.hestf; b.hestf; heltfx1];
v2preamble2 = [b.lstf; lx.lltfx; b.lsig; b.hesiga; b.hestf; b.hestf; heltfx2];
write_complex_binary(scaling*v2preamble1, ...
    sprintf('%s/%s/%s/ncjt_v2preamble_ue1.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*v2preamble2, ...
    sprintf('%s/%s/%s/ncjt_v2preamble_ue2.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*[v2preamble1, v2preamble2], ...
    sprintf('%s/%s/%s/ncjt_v2preamble.bin',datadir,mimotype,modtype));

% Save LTF for debugging
write_complex_binary(scaling*heltfx1, ...
    sprintf('%s/%s/%s/heltfx_ue1.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*heltfx2, ...
    sprintf('%s/%s/%s/heltfx_ue2.bin',datadir,mimotype,modtype));

% Save binary data
fid = fopen(sprintf('%s/%s/%s/enc_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/strm_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);
fid = fopen(sprintf('%s/%s/%s/strm_data_s1.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:,1), "char");
fclose(fid);
fid = fopen(sprintf('%s/%s/%s/strm_data_s2.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:,2), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/tx_strm_data.bin',datadir,mimotype,modtype),'wb');
txstrmdata = strmdata.';
fwrite(fid, txstrmdata, "char");
fclose(fid);

txdsyms = txdsyms(cfg.scInd, :, :);
txdsyms_tmp = reshape(permute(txdsyms, [3, 1, 2]), [], 1);
write_complex_binary(txdsyms_tmp, ...
    sprintf("%s/%s/%s/txdsyms.bin",datadir,mimotype,modtype));

fprintf("\n")

