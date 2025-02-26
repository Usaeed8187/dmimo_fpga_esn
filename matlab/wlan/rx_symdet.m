function [y, CSI] = rx_symdet(x, chanEst, noiseVar)
% LMMSE MIMO symbol detection

[Ndsc, numSS, numRx] = size(chanEst);
numSym = size(x ,2);

CSI = zeros(Ndsc, numSS);

y = complex(zeros(size(x, 1), numSym, numSS));
for idx = 1:size(chanEst, 1)
	H = reshape((chanEst(idx,:,:)), numSS, numRx);
    invH = inv(H*H'+noiseVar*eye(numSS));
    CSI(idx, :)  = 1./real(diag(invH));
    y(idx, :, 1:numSS) = reshape(x(idx, :, :), numSym, numRx) * (H' * invH);  %#ok
end


end % EOF
