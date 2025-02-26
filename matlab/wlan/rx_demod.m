function [rxData, chanEstData, nEst, pwrEst, cpePhi] = rx_demod(cfg, rx, chanEst)
% OFDM demodulation and CPE estimation
%
% By Donald Liang, last updated Jan 30, 2024


[~, Nsts, Nr] = size(chanEst); % number of streams and receiver antennas
Np = numel(cfg.pilotInd); % number of tracking pilots

chanEstData = chanEst(cfg.scdIdx,:,:);
chanEstPilots = chanEst(cfg.scpIdx,:,:);

rxDemod = ofdm_demod(cfg, rx);
Nsym = size(rxDemod,2);

rxData = rxDemod(cfg.scdIdx,:,:);
rxPilots = rxDemod(cfg.scpIdx,:,:);

refPilots = gen_pilotseq(cfg.Nsd, Nsym, Nsts);


% CPE and noise estimation
nEstSum = 0;
pwrEstSum = 0;
cpePhi = zeros(Nsym, 1);

for n = 1:Nsym
    cpeTemp = complex(zeros(Np, 1));
    estPilots = complex(zeros(Np, Nr));
    for r = 1:Nr
        % sum over receiver antennas
        for s = 1:Nsts
            % sum over streams
            estPilots(:,r) = estPilots(:,r) + chanEstPilots(:,s,r) .* refPilots(:,n,s);
        end
        cpeTemp = cpeTemp + rxPilots(:,n,r) .* conj(estPilots(:,r));
    end
    % sum over pilot tones
    phi = angle(sum(cpeTemp));
    cpePhi(n) = phi;
    
    % CPE compensation
    rxPilotsCPE = exp(-1i * phi) * squeeze(rxPilots(:,n,:));
    rxData(:,n,:) = exp(-1i * phi) * rxData(:,n,:);
    
    % noise estimate
    pilotError = estPilots - rxPilotsCPE;
    nEstSum = nEstSum + mean(real(pilotError(:).*conj(pilotError(:))));

    % power estimate
    pwrEstSum = pwrEstSum + mean(estPilots(:).*conj(estPilots(:)));
end

% Average power and noise estimation
pwrEst = pwrEstSum/Nsym;
nEst = nEstSum/Nsym;
nEst = sqrt(pwrEst/2.0)*nEst/Nsym;


end % EOF
