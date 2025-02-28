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

if strcmpi(cfg.mode,'2t1s_svd')
    x = dsyms_shifted;
    x_precoded = zeros([size(x) cfg.Nt]);
    
    r1 = read_complex_binary(cfg.csi_dir);
    h1 = reshape(r1, cfg.Nr, cfg.Nt, cfg.Nsc, []);

    if cfg.wideband
        h_current = mean(h1, [3,4]);

        [~, ~, V] = svd(h_current);
        precoding_vector = V(:,1);
        % precoding_vector = precoding_vector ./ sqrt(var(precoding_vector(:)));
        % precoding_vector = sign(precoding_vector(1,1)) .* precoding_vector;

        max_len = min(size(h1, 4), size(x, 2));
        for channel_idx=1:max_len
            for sc_ind = 1:size(x,1)
                x_current = x(sc_ind, channel_idx);    
                x_precoded(sc_ind, channel_idx, :)= precoding_vector * x_current;
            end
        end
        dsyms_mapped = x_precoded;
        Nt = cfg.Nt;
    else
    
        max_len = min(size(h1, 4), size(x, 2));
        for channel_idx=1:max_len
    
            for sc_ind = 1:cfg.Nsc
                h_current = squeeze(h1(:, :, sc_ind, channel_idx));
                x_current = x(sc_ind, channel_idx);
    
                [~, ~, V] = svd(h_current);
                precoding_vector = V(:,1);
                % precoding_vector = sign(precoding_vector(1,1)) .* precoding_vector;
    
                x_precoded(sc_ind, channel_idx, :)= precoding_vector * x_current;
    
            end
        end
        dsyms_mapped = x_precoded;
        Nt = cfg.Nt;
    end
else
    % Spatial mapping
    if cfg.spatialExpansion
        Nt = 2 * cfg.Nss;
        dsyms_mapped = spatialMapping(dsyms_shifted, cfg);
    else
        Nt = cfg.Nss;
        dsyms_mapped = dsyms_shifted;
    end
end

% dsyms_mapped_rg = zeros(cfg.Nfft,size(dsyms_mapped,2), cfg.Nt);
% dsyms_mapped_rg(cfg.scInd, :, :) = dsyms_mapped;

% OFDM modulation 
txsig = ofdm_mod(cfg, dsyms_mapped, true);

% Signal power normalization
normFactor = cfg.Nfft/sqrt(cfg.Nss*cfg.Nsc);
txsig = normFactor * reshape(txsig, [], Nt);

end % EOF
