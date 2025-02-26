function [data, encdata, eqDataSym, csiData, sigPwrEst, nVarEst] = rx_processing(cfg, htltf, htdata)
% Wi-Fi receiver processing
%
% By Donald Liang, last updated Aug 22, 2023
  
% Demodulate and perform channel estimation
htltfDemod = ofdm_demod(cfg, htltf);
chanEst = ltf_chan_est(cfg, htltfDemod);

% Estimate the noise power in data symbols
% [nVarHT, nSigHT] = noise_est(cfg, htdata, chanEst);
% snr = 10*log10(nSigHT/nVarHT);
% fprintf('Data noise variance: %g (SNR %f)\n', nVarHT, snr);

% Demode data symbols
[rxData, chanEstData, nVarEst, sigPwrEst] = rx_demod(cfg, htdata, chanEst);

% MMSE equalization
[eqDataSym, csiData] = rx_symdet(rxData, chanEstData, nVarEst);

% LDPC tone interleaving
if cfg.Nsc == 242
    eqDataSym = eqDataSym(cfg.intlv,:,:);
    csiData = csiData(cfg.intlv,:,:);
end

if strcmpi(cfg.cctype, 'BCC')
    % Soft-LLR QAM demapping
    qamllr = qam_llrmetric(eqDataSym, cfg.Mord, csiData);

    % Data deinterleaving and stream demapping
    sdatallr = bcc_deinterleave(cfg, qamllr);
    encdatallr = reshape(sdatallr.', [], 1);
	encdata = (encdatallr > 0);

    % BCC decoding
    decdata = bcc_decode(encdatallr);
else
    % Soft-LLR QAM demapping
    llrdata = (-1.0/nVarEst) * qam_llrmetric(eqDataSym, cfg.Mord, csiData);

    % Data stream demapping
    llrdata = reshape(llrdata, [], cfg.Nss);
    llrdata = reshape(llrdata.', [], 1);
    encdata = (llrdata < 0);

    % LDPC decoding
    decllr = ldpc_decode(llrdata, cfg);
    decdata = (decllr > 0);
end

% descramble
scrdata = psdu_scramble(decdata);

% extract PSDU
data = scrdata(16 + (1:cfg.PSDUlen*8));

end % EOF


