% System configuration
modtype = "QPSK"; % QPSK/16QAM/64QAM
PSDULength = 1036; % use 1090/1036 for 85/80 data symbols
cfg = sys_config('2x2', PSDULength, 2);
save_txsig = true;

% Load 802.11 beacon signals
% load('beacon2x2.mat')

% LTF reference signals (freq-domain)
ltfRef = [1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, ...
       1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, ...
       1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, ...
       1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,-1,-1 ].';

% LTFs for 2x2 case
ltf2x2 = cat(3, [ltfRef, -ltfRef], [ltfRef, ltfRef]);
ltfdata = zeros(cfg.Nfft,2,2);
ltfdata(cfg.scInd,:,:) = ltf2x2;
%ltfdata = cyclic_shift(ltfdata);
ltf2x2 = ltfdata(cfg.scInd,:,:);

% generate LTF signal (time domain)
normFactor = cfg.Nfft/sqrt(cfg.Nt*cfg.Nsc);
ltfx = normFactor * ofdm_mod(cfg, ltfdata);
% d = ltfx - reshape(htltf, 80, 2, 2);
% assert(max(abs(d(:))) == 0, "LTF generation not OK")
htltfx = reshape(ltfx, [], 2);

save('htltfx_nocs.mat', 'htltfx');
write_complex_binary(htltfx, './data/2x2/htltfx_nocs.bin')
