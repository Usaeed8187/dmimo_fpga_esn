function [y, CSI] = rx_equalize(x, chanEst, noiseVar)

[Ndsc, numTx, numRx] = size(chanEst);
numSym = size(x ,2);

CSI = zeros(Ndsc, numTx);

y = complex(zeros(size(x, 1), numSym, numTx));
for idx = 1:size(chanEst, 1)
	H = reshape((chanEst(idx,:,:)), numTx, numRx);
    invH = inv(H*H'+noiseVar*eye(numTx));
    CSI(idx, :)  = 1./real(diag(invH));
    y(idx, :, 1:numTx) = reshape(x(idx, :, :), numSym, numRx) * (H' * invH);  %#ok<MINV>
end


