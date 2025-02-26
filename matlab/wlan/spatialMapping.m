function dsyms_mapped = spatialMapping(dsyms, cfg)    
% Sptial mapping (precoding)

[Nfft, Nsyms, ~] = size(dsyms);
% dataInd = [-28:-1 1:28] + 33;
dataInd = [5:11 13:25 27:32 34:39 41:53 55:61];
pilotInd = [12,26,40,54];
dsyms_mapped = complex(zeros(Nfft, Nsyms, cfg.Nt));
for n=1:length(dataInd)
    dix = dataInd(n);
    csd = cfg.spatialMapping(:,:,n);
    dsyms_mapped(dix,:,:) = squeeze(dsyms(dix,:,:)) * csd;
end
dsyms_mapped(pilotInd,:,1:2) = dsyms(pilotInd,:,:);
dsyms_mapped(pilotInd,:,3:4) = dsyms(pilotInd,:,:);
end % EOF
