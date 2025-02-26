function [txsig, encdata, sdata_intlv, dsyms_shifted, dsyms_mapped] = tx_processing(cfg, psdu)
% Wi-Fi transmitter processing
%
% By Donald Liang, last updated July 30, 2023

% PSDU data padding
if strcmpi(cfg.cctype, 'BCC')
    data = [zeros(16,1); psdu(:); zeros(6,1); zeros(cfg.PadBits, 1)];
else
    data = [zeros(16,1); psdu(:)];
end

% scrambling
scrdata = psdu_scramble(int8(data));

if strcmpi(cfg.cctype, 'BCC')
    % BCC encoding
    scrdata(16+length(psdu)+(1:6)) = zeros(6,1);  % zero-out tail bits
    encdata = bcc_encode(scrdata);
else
    % LDPC encoding
    encdata = ldpc_encode(cfg, scrdata);
    % PostFEC padding
    pbits = genPadding(cfg.numPadding);
    encdata = [encdata(:); pbits];
end

% Stream mapping
tmpdata = reshape(encdata, cfg.Nss, []);
sdata = transpose(tmpdata);

% Data interleaving for BCC
if strcmpi(cfg.cctype, 'BCC')
    sdata_intlv = bcc_interleave(cfg, sdata);
else
    sdata_intlv = sdata;
end

% Constellation mapping
datasyms = zeros(size(sdata_intlv,1)/cfg.Mord, cfg.Nss);
for k=1:cfg.Nss
    datasyms(:,k) = qam_mapping(sdata_intlv(:,k), cfg.Mord);
end
datasyms = reshape(datasyms, cfg.Nsd, [], cfg.Nss);

% LDPC tone interleaving
dsyms_intlv = datasyms;
% if cfg.Nfft==256 && strcmpi(cfg.cctype, 'LDPC')
%     dsyms_intlv(cfg.intlv,:,:) = datasyms;
% end

% Insert pilot tones
numSyms = size(dsyms_intlv, 2);
pilotsyms = gen_pilotseq(cfg.Nsd, numSyms, cfg.Nss);
dsyms_mapped = zeros(cfg.Nfft, numSyms, cfg.Nss);
dsyms_mapped(cfg.dataInd, :, :) = dsyms_intlv;
dsyms_mapped(cfg.pilotInd, :, :) = pilotsyms;

% Cyclic shift and direct spatial mapping
dsyms_shifted = cyclic_shift(dsyms_mapped, cfg.Nfft);

% Spatial mapping
if cfg.spatialExpansion
    Nt = 2 * cfg.Nss;
    dsyms_mapped = spatialMapping(dsyms_shifted, cfg);
else
    Nt = cfg.Nss;
    dsyms_mapped = dsyms_shifted;
end

% OFDM modulation 
txsig = ofdm_mod(cfg, dsyms_mapped, true);

% Signal power normalization
normFactor = cfg.Nfft/sqrt(cfg.Nss*cfg.Nsc);
txsig = normFactor * reshape(txsig, [], Nt);

end % EOF
