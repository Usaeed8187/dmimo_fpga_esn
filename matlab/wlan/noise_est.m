function [nest, pwrest] = noise_est(cfg, rx, chanEst)
% Average noise power estimation
%
% By Donald Liang, last updated July 28, 2021


[~, Nsts, Nr] = size(chanEst);

Np = 4;
pilotInd = [8, 22, 35, 49];
chanEstPilots = chanEst(pilotInd,:,:);

rxDemod = ofdm_demod(cfg, rx);
rxPilots = rxDemod(pilotInd,:,:);
Nsym = size(rxDemod,2);

offset = 3;
refPilots = gen_pilotseq(Nsym, Nsts, offset);


% CPE and noise estimation
nest = 0;
pwrest = 0;

for n = 1:Nsym
    cpe_temp = complex(zeros(Np, 1));
    estPilots = complex(zeros(Np, Nr));
    for r = 1:Nr
        % sum over receiver antennas
        for s = 1:Nsts
            % sum over streams
            estPilots(:,r) = estPilots(:,r) + chanEstPilots(:,s,r) .* refPilots(:,n,s);
        end
        cpe_temp = cpe_temp + rxPilots(:,n,r) .* conj(estPilots(:,r));
    end
    % sum over pilot tones
    cpe = angle(sum(cpe_temp));
    
    % CPE compensation
    rxPilotsCPE = exp(-1i * cpe) .* squeeze(rxPilots(:,n,:));
    
    % noise estimate
    pilotError = estPilots - rxPilotsCPE;
    nest = nest + mean(real(pilotError(:).*conj(pilotError(:))));

    % power est
    pwrest = pwrest + mean(estPilots(:).*conj(estPilots(:)));
end

nest = nest/Nsym;
pwrest = pwrest/Nsym;

