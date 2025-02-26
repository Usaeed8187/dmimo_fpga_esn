function y = ofdm_demod(cfg, rx, offset)
% Demodulate time-domain signals
%
% Inputs:
%   cfg     system configuration
%   rx      received time-domain signals (symLen * Nsym, Nr)
%   offset  OFDM symbol offset
% Output:
%   y       demodulated frequency-domain signals (Nsc, Nsym, Nr)

% OFDM symbol offset
if nargin < 3
    offset = round(cfg.Ncp * 0.75);  % default 12 samples
end

% reshape inputs
rxd = reshape(rx, (cfg.Nfft + cfg.Ncp), [], cfg.Nr);

% remove CP
%fftIn = rxd(offset+1:cfg.Nfft+offset, :, :);
dix = [cfg.Ncp+1:cfg.Nfft+offset, offset+1:cfg.Ncp];
fftIn = rxd(dix, :, :);

% FFT and shift
normFactor = sqrt(cfg.Nt * cfg.Nsc) / cfg.Nfft;
fftOut = fft(fftIn, cfg.Nfft, 1);
yf = fftshift(normFactor*fftOut, 1);

% remove void subcarriers and DC
y = yf(cfg.scInd, :, :);


end % EOF
