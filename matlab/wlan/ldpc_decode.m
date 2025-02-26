function [decoded,decllr,decok]=ldpc_decode(llrdata, cfg)

% fixed coding length and rate
% LDPC input block length
ldpcK = cfg.ldpcLen * cfg.rate;
ldpcN = cfg.ldpcLen;
numCW = cfg.numCW;
maxIters = 20;
decok = true;

% Calculate bit lengths for source and codeword
shrtBits = bitpattern(cfg.numSHRT, numCW);
puncBits = bitpattern(cfg.numPuncBits, numCW);
repBits = bitpattern(cfg.numRepBits, numCW);   

% Prepare LDPC decoder input buffers
decIn = zeros(ldpcN, numCW);
offset = 0;
for k=1:cfg.numCW
    datalen = ldpcK-shrtBits(k);
    chklen = ldpcN-ldpcK-puncBits(k);
    decIn(1:datalen, k) = llrdata(offset+(1:datalen));
    decIn(ldpcK+(1:chklen), k) = llrdata(offset+datalen+(1:chklen));
    if repBits(k) > 0
        decIn(1:repBits(k),k) = 0.5*(llrdata(offset+(1:repBits(k))) + ...
                                    llrdata(offset+datalen+chklen+(1:repBits(k))));
    end
    offset = offset+datalen+chklen+repBits(k);
end

[decout, iters, pchks] = ldpcDecodeCore(decIn, cfg.rate, 3, [0.5,0.5], maxIters, true, ...
        'soft decision','whole codeword');
if any(iters >= maxIters & sum(pchks) > 0)
    decok = false;
end

% Decoder LLR for iterative processing
decllr = zeros(ldpcN*numCW+cfg.numRepBits, 1);
offset = 0;
for k=1:numCW
    decllr(offset+(1:ldpcN)) = decout(:,k);
    decllr(offset+ldpcN+(1:repBits(k))) = decout(1:repBits(k),k);
    offset = offset+ldpcN+repBits(k);
end

% Codeword output
totalBits = ldpcK*numCW-cfg.numSHRT;
decoded = zeros(totalBits, 1, 'int8');
offset = 0;
for k=1:numCW
    datalen = ldpcK-shrtBits(k);
    decoded(offset+(1:datalen)) = int8(decout(1:datalen,k) <= 0);
    offset = offset+datalen;
end

end % ldpc_decode


function [bitPat] = bitpattern(totalBits, nCW)
    nMin = floor(totalBits/nCW);
    nExtra = rem(totalBits,nCW);
    bitPat = repmat(nMin, 1, nCW);
    bitPat(1:nExtra) = bitPat(1:nExtra) + 1;
end
