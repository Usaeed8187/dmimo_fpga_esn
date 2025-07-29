function su_mimo_sigen_he(cfg, mimotype, psdulen, modtype, datadir)

% Create data output folder
[~, ~, ~] = mkdir(fullfile(datadir, mimotype, modtype)); % create output folder

% Load 802.11 beacon signals
if strcmpi(mimotype, "2t2s")
    b = load('beacon2x2he.mat');
    bx = load('heltfx.mat');
elseif strcmpi(mimotype, "4t4s")
    b= load('beacon4x4he.mat');
    bx = load('heltfx4.mat');
else
   error('Unkown MIMO setup')
end

% Set random substream
stream = RandStream('mt19937ar','Seed',2201203);
RandStream.setGlobalStream(stream);

% Generate packet data
txPSDU = randi([0 1], psdulen*8, 1, 'int8'); % PSDULength in bytes

% Generate data symbols
[txdata, encdata, strmdata, txdsyms] = tx_processing(cfg, txPSDU);

% scaling for Nsc=140
ltf_scaling = sqrt(242/cfg.Nsc);
bx.heltfx = ltf_scaling * bx.heltfx;

% Time-domain preamble signals (HE-SIGA, HE-STF, HE-LTF)
preamble = [b.lstf; b.lltf; b.lsig; b.hesiga; b.hestf; b.hestf; bx.heltfx];

% Prepare transmitter signal for USRP
txFrame = reshape([preamble; txdata], [], cfg.Nt);

% Convert Tx signals to range (-1,1)
% scaling = 1.0/max(abs([real(txFrame(:)); imag(txFrame(:))]));
scaling = 0.5; % fixed scaling
txsig = signal_clipping(scaling * txFrame);

% Zero-padding for burst transmission
padding = zeros(500, cfg.Nt);
txsig = [padding; txsig;]; % padding];

% Show system parameters
nLegacyPreamble = 5;
nHEPreamble = (size(preamble,1) - 80*nLegacyPreamble) / (cfg.Nfft+cfg.Ncp);
nDataSymbols = size(txdata, 1) / (cfg.Nfft+cfg.Ncp);
fprintf("Number of HE symbols: %d\nNumber of Data symbols: %d\n", ...
        nHEPreamble, nDataSymbols);
fprintf("Tx signal frame size: %d\n", size(txsig,1));
fprintf("Tx frame data length (bits): %d\n", size(strmdata,1));
fprintf("Tx signal scaling: %.15f\n", scaling);

% Save data fro GNU Radio implementation
fprintf("Saving GNU Radio Tx data files ...\n")
write_complex_binary(txsig, ...
        sprintf('%s/%s/%s/txsig_all.bin',datadir,mimotype,modtype));
for k=1:cfg.Nt
    write_complex_binary(txsig(:,k), ...
        sprintf('%s/%s/%s/txsig_s%d.bin',datadir,mimotype,modtype,k));
end

% Save preamble signals
write_complex_binary(scaling*preamble, ...
    sprintf('%s/%s/%s/preamble.bin',datadir,mimotype,modtype));
write_complex_binary(scaling*preamble(1:end-cfg.SymLen,:), ...
    sprintf('%s/%s/%s/preamble_noltfx.bin',datadir,mimotype,modtype));

% Generate LTF for precoding
ltfRef = [-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1, ...
         1,-1,1,1,1,1,-1,1,-1,-1,1,1,-1,1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,1,-1,1,1,-1,-1,-1,-1,1, ...
         -1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,1,1, ...
         -1,1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,-1,1,-1,1,-1,1,1,-1, ...
         1,1,1,-1,-1,1,-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1, ...
         -1,-1,-1,1,-1,1,-1,-1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,-1,-1,1,1,-1,1,1,1,1,1,1,1,-1,1, ...
         1,-1,-1,-1,-1,1,-1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,1,1,1,1,-1,-1,1,1,1,1,1,-1, ...
         1,1,-1,-1,-1,1,-1,-1,-1,1,-1,1,-1,1,1].';
ltf2x2 = cat(3, [ltfRef, -ltfRef], [ltfRef, ltfRef]);
if (size(ltfRef, 1) ~= cfg.Nsc)
    npos = (size(ltfRef, 1) - cfg.Nsc)/2;
    ltf2x2 = ltf2x2(npos+1:end-npos, :, :);
end
ltf2x2 = reshape(ltf2x2, [], 2);
write_complex_binary(ltf2x2, ...
    sprintf('%s/%s/%s/ltf2x2.bin',datadir,mimotype,modtype));
write_complex_binary(ltf2x2(:,1), ...
    sprintf('%s/%s/%s/ltf2x2_t1.bin',datadir,mimotype,modtype));
write_complex_binary(ltf2x2(:,2), ...
    sprintf('%s/%s/%s/ltf2x2_t2.bin',datadir,mimotype,modtype));

% Save LTF for debugging
write_complex_binary(scaling*bx.heltfx, ...
    sprintf('%s/%s/%s/heltfx.bin',datadir,mimotype,modtype));

% Save binary data
fid = fopen(sprintf('%s/%s/%s/enc_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, encdata(:), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/strm_data.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:), "char");
fclose(fid);

fid = fopen(sprintf('%s/%s/%s/strm_data_s1.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:, 1), "char");
fclose(fid);
fid = fopen(sprintf('%s/%s/%s/strm_data_s2.bin',datadir,mimotype,modtype),'wb');
fwrite(fid, strmdata(:, 2), "char");
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

