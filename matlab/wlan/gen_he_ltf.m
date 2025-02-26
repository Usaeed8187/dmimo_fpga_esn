% System configuration
modtype = "QPSK"; % QPSK/16QAM/64QAM
PSDULength = 847; % use 484 for QPSK, 970 for 16QAM, 1456 for 64QAM
cfg = sys_config_he('2x2', PSDULength, modtype, 'LDPC');

% Load 802.11 beacon signals
load('beacon2x2he.mat')

% LTF reference signals (freq-domain)
ltfRef = [-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1, ...
         1,-1,1,1,1,1,-1,1,-1,-1,1,1,-1,1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,1,-1,1,1,-1,-1,-1,-1,1, ...
         -1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,1,1, ...
         -1,1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,-1,1,-1,1,-1,1,1,-1, ...
         1,1,1,-1,-1,1,-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1, ...
         -1,-1,-1,1,-1,1,-1,-1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,-1,-1,1,1,-1,1,1,1,1,1,1,1,-1,1, ...
         1,-1,-1,-1,-1,1,-1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,1,1,1,1,-1,-1,1,1,1,1,1,-1, ...
         1,1,-1,-1,-1,1,-1,-1,-1,1,-1,1,-1,1,1].';
% for Nsc=244
% ltfRef = [1; ltfRef; -1];

% LTFs for 2x2 case, P matrix = [1, -1; 1, 1]
ltf2x2 = cat(3, [ltfRef, -ltfRef], [ltfRef, ltfRef]);
ltfdata = zeros(cfg.Nfft,2,2);
ltfdata(cfg.scInd,:,:) = ltf2x2;
% if length(ltfRef) == 242
%     % Q matrix is different for CPT, Q = [1, -1; 1, -1]
%     ltfdata(cfg.pilotInd,2,2) = - ltfdata(cfg.pilotInd,2,2);
% end

ltfdata = cyclic_shift(ltfdata, cfg.Nfft);
ltf2x2 = ltfdata(cfg.scInd,:,:);

% generate LTF signal (time domain)
normFactor = cfg.Nfft/sqrt(cfg.Nt*cfg.Nsc);
ltfx = normFactor * ofdm_mod(cfg, ltfdata);
heltfx = reshape(ltfx, [], 2);
% if length(ltfRef) == 242
%     d = heltf - heltfx; 
%     assert(max(abs(d(:))) <= 1e-8, "LTF generation not OK")
% end

save('heltfx.mat','heltfx')
