function y = cyclic_shift(x, nfft)
% apply cyclic shifts to spatial streams

% number of spatial streams
Nss = size(x,3);
assert(Nss == 1 || Nss == 2 || Nss == 4 || Nss == 6)

% FFT size
if nargin < 2
    nfft = 64;
end

% cyclic shifts constants
cshift = [0; -8; -4; -12; -7; -13];

% phase steps for all subcarriers
k = (1:nfft).' - nfft/2 - 1;

y = zeros(size(x),'like',x);
for ss=1:Nss
    phaseshift = exp(-2i*pi*cshift(ss)/nfft*k);
    y(:,:,ss) = x(:,:,ss) .* phaseshift;
end


end % EOF
