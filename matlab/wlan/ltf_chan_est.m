function hest = ltf_chan_est(cfg, rx)
% Channel estimation using LTFs
%
% Inputs:
%  cfg    system config
%  rx     domodulated LTF (scNum, nLTF, nRx)
%
% Output:
%  hest   channel estimation (scNum, nSTS, nRx)
%
% By Donald Liang, last updated Nov 2, 2023

% system params
nSTS = cfg.Nss;
[scNum, nLTF, nRx] = size(rx);

% LTF reference signals
if cfg.Nsc == 56  % HT-LTF
ltf = [1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, ...
       1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, ...
       1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, ...
       1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,-1,-1 ].';
elseif cfg.Nsc == 242  % HE-LTF
ltf = [-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1, ...
         1,-1,1,1,1,1,-1,1,-1,-1,1,1,-1,1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,1,-1,1,1,-1,-1,-1,-1,1, ...
         -1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,1,1, ...
         -1,1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,-1,1,-1,1,-1,1,1,-1, ...
         1,1,1,-1,-1,1,-1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,-1, ...
         -1,-1,-1,1,-1,1,-1,-1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,-1,-1,1,1,-1,1,1,1,1,1,1,1,-1,1, ...
         1,-1,-1,-1,-1,1,-1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,1,1,1,1,-1,-1,1,1,1,1,1,-1, ...
         1,1,-1,-1,-1,1,-1,-1,-1,1,-1,1,-1,1,1].';
else
    error('LTF mode unsupported')
end

P = [1, -1, 1, 1; 1, 1, -1, 1; 1, 1, 1, -1; -1, 1, 1, 1];
ltfnorm = (1/nLTF) .* ltf;

hest = complex(zeros(scNum, nSTS, nRx));
Pd = P(1:nSTS, 1:nLTF);
for k = 1:scNum % for all subcarriers
    hest(k, :, :) = Pd * shiftdim(rx(k, :, :), 1) .* ltfnorm(k);
end

% interpolation for HE-LTE pilots
% Q matrix is different for CPT, Q = [1, 1; 1, -1]
% CPT pilots are use for tracking CPE
if cfg.Nsc == 242
    hmag = abs(hest(cfg.scdIdx,:,:));
    hphase = unwrap(angle(hest(cfg.scdIdx,:,:)));
    hh = interp1(cfg.scdIdx.', cat(4,hmag,hphase), (1:cfg.Nsc).', 'linear', 'extrap');
    hest = hh(:,:,:,1).*exp(1i*hh(:,:,:,2));
end

% EOF
