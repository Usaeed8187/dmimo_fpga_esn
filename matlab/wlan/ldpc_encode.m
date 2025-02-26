function [encoded] = ldpc_encode(cfg, srcdata)
% Input:
%   srcdata     source input bits (int8)
% Output:
%   encoded     codewords
%

% LDPC input block length
ldpcK = cfg.ldpcLen * cfg.rate;
ldpcN = cfg.ldpcLen;
numCW = cfg.numCW;

% Calculate bit lengths for source and codeword
shrtBits = bitpattern(cfg.numSHRT, numCW);
puncBits = bitpattern(cfg.numPuncBits, numCW);
repBits = bitpattern(cfg.numRepBits, numCW);   
totalBits = ldpcN*numCW + cfg.numRepBits - cfg.numSHRT - cfg.numPuncBits;

% Prepare LDPC encoder input buffer
encIn = zeros(ldpcK, numCW);
startPos = 0;
for n=1:numCW
    inputLen = ldpcK-shrtBits(n);
    inputData = srcdata(startPos+(1:inputLen));
    encIn(:,n) = [inputData(:);zeros(shrtBits(n),1,'int8')];
    startPos = startPos + inputLen;
end

% LDPC encoding
chkBits = ldpcEncodeCore(encIn, cfg.rate);

% Codeword output buffer
encoded = zeros(totalBits, 1, 'int8');
startPos = 0;
outPos = 0;
for k=1:numCW
    inputLen = ldpcK-shrtBits(k);
    inputData = srcdata(startPos+(1:inputLen));
    if (cfg.numPuncBits > 0) % puncturing
        outData = [inputData;chkBits(1:end-puncBits(k),k)];
    else % repeat source bits
        outData = [inputData;chkBits(:,k);inputData(1:repBits(k))];
    end
    outlen = length(outData);
    encoded(outPos+(1:outlen)) = outData;
    outPos = outPos + outlen;
    startPos = startPos + inputLen;
end

end % ldpc_encode


function [bitPat] = bitpattern(totalBits, nCW)
    nMin = floor(totalBits/nCW);
    nExtra = rem(totalBits,nCW);
    bitPat = repmat(nMin, 1, nCW);
    bitPat(1:nExtra) = bitPat(1:nExtra) + 1;
end

