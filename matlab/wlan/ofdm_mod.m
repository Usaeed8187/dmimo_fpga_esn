function y = ofdm_mod(cfg, x, windowing)
% OFDM modulate

if nargin < 3
    windowing = false;
end

% system params
fftLen = cfg.Nfft; % default 64
cpLen = cfg.Ncp; % default 16

% shift and IFFT
fftIn = ifftshift(x, 1);
fftOut = ifft(fftIn,fftLen, 1);

% add CP
cp = fftOut(fftLen-cpLen+1:fftLen, :, :);
y = [cp; fftOut];

% windowing (winSize = 2)
if windowing
    y(1,2:end,:) = 0.5*(y(1,2:end,:) + y(cpLen+1,1:end-1,:));  % postfix
end

end
