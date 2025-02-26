function [y, actualNumIter, finalParityChecks] = ldpcDecodeCore(x, rate, algChoice, alphaBeta, ...
    maxNumIter, earlyTermination, varargin)
%ldpcDecodeCore Decode LDPC encode data
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Y = ldpcDecodeCore(X,RATE,ALGCHOICE,ALPHABETA,MAXNUMITER,EARLYTERMINATION)
%   decodes the log-likelihood ratios X using a WLAN LDPC code at the
%   specified rate (rate). Y are hard-decision outputs for the information
%   bits of the codewords.
%
%   Each column of X specifies the log-likelihood ratios of a codeword. The
%   number of rows in X should be equal to one of the four valid block
%   lengths: 648, 672, 1296, and 1944.
%
%   RATE must be equal to '1/2', '2/3', '3/4', '5/6', '5/8' and '13/16'.
%
%   ALGCHOICE specifies the LDPC decoding algorithm as one of these values:
%     0 - Belief Propagation
%     1 - Layered Belief Propagation
%     2 - Layered Belief Propagation with Normalized Min-Sum approximation
%     3 - Layered Belief Propagation with Offset Min-Sum approximation
%
%   ALPHABETA specifies the scaling factor for Normalized Min-Sum
%   approximation or the offset factor for Offset Min-Sum approximation.
%   Its value is irrelevant for the other two LDPC algorithms but still
%   needed.
%
%   MAXNUMITER specifies the number of decoding iterations required to
%   decode the input X.
%
%   EARLYTERMINATION specifies if the conditions for an early termination
%   should be checked. If true, after each iteration, ldpcDecodeCore will
%   determine independently for each column of X if all parity checks are
%   satisfied, and will stop for column(s) with all parity checks
%   satisfied; otherwise, the function will execute for the maximum number
%   of iterations MAXNUMITER.
%
%   See https://www.mathworks.com/help/comm/ref/ldpcdecoder.html for the
%   definition of log-likelihood ratios and details about the decoding
%   algorithm.
%
%   The following optional character vector inputs are accepted in varargin:
%   'whole codeword' - If 'whole codeword' is provided, Y contains
%                      whole Y codewords and the number of rows is equal to
%                      the number of rows in X; otherwise, Y contains the
%                      information bits of the codewords only and the
%                      number of rows is equal to the number of information
%                      bits in one codeword.
%   'soft decision'  - If 'soft decision' is provided, each element in
%                      Y is a log-likelihood ratio for the corresponding Y
%                      bit and is of type double; otherwise, each element
%                      is either 0 or 1 and is of type int8.
%
%   [Y,ACTUALNUMITER,FINALPARITYCHECK] = ldpcDecodeCore(...)
%   provides optional outputs.
%
%   ACTUALNUMITER is a row vector of positive integer(s) whose length is
%   equal to the number of columns of X. The i-th element corresponds to
%   the actual number of decoding iterations executed for the i-th column
%   of X.
%
%   FINALPARITYCHECK is a matrix that holds the final parity checks. The
%   i-th column corresponds to the final parity checks for the i-th
%   codeword. The number of rows is equal to the number of parity-check
%   bits in a codeword. If all elements in FINALPARITYCHECK are zeros, Y
%   corresponds to a valid codeword.
%
%   See also ldpcEncodeCore, ldpcMatrix.

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

[codewordLength, numCodewords] = size(x);
switch codewordLength
    case 648
        Z = 27; % IEEE Std 802.11-2012, Table F-1
        if strcmp(rate, '1/2') || isequal(rate, 1/2)
            [infoLen, offsetWeight, columnIndexMap] = d_1_2_648params(codewordLength);
        elseif strcmp(rate, '2/3') || isequal(rate, 2/3)
            [infoLen, offsetWeight, columnIndexMap] = d_2_3_648params(codewordLength);
        elseif strcmp(rate, '3/4') || isequal(rate, 3/4)
            [infoLen, offsetWeight, columnIndexMap] = d_3_4_648params(codewordLength);
        else % case '5/6'
            [infoLen, offsetWeight, columnIndexMap] = d_5_6_648params(codewordLength);
        end
    case 1296
        Z = 54; % IEEE Std 802.11-2012, Table F-2
        if strcmp(rate, '1/2') || isequal(rate, 1/2)
            [infoLen, offsetWeight, columnIndexMap] = d_1_2_1296params(codewordLength);
        elseif strcmp(rate, '2/3') || isequal(rate, 2/3)
            [infoLen, offsetWeight, columnIndexMap] = d_2_3_1296params(codewordLength);
        elseif strcmp(rate, '3/4') || isequal(rate, 3/4)
            [infoLen, offsetWeight, columnIndexMap] = d_3_4_1296params(codewordLength);
        else % case '5/6'
            [infoLen, offsetWeight, columnIndexMap] = d_5_6_1296params(codewordLength);
        end
     case 1944
        Z = 81; % IEEE Std 802.11-2012, Table F-3
        if strcmp(rate, '1/2') || isequal(rate, 1/2)
            [infoLen, offsetWeight, columnIndexMap] = d_1_2_1944params(codewordLength);
        elseif strcmp(rate, '2/3') || isequal(rate, 2/3)
            [infoLen, offsetWeight, columnIndexMap] = d_2_3_1944params(codewordLength);
        elseif strcmp(rate, '3/4') || isequal(rate, 3/4)
            [infoLen, offsetWeight, columnIndexMap] = d_3_4_1944params(codewordLength);
        else % '5/6'
            [infoLen, offsetWeight, columnIndexMap] = d_5_6_1944params(codewordLength);
        end
    otherwise
       % For block size 672, use for DMG
       Z = 42; % IEEE Std 802.11ad-2012, Tables 21-6, 21-7, 21-8, 21-9
       if strcmp(rate, '1/2') || isequal(rate, 1/2)
            [infoLen, offsetWeight, columnIndexMap] = d_1_2_672params(codewordLength);
       elseif strcmp(rate, '3/4') || isequal(rate, 3/4)
            [infoLen, offsetWeight, columnIndexMap] = d_3_4_672params(codewordLength);
       elseif strcmp(rate, '5/8') || isequal(rate, 5/8)
            [infoLen, offsetWeight, columnIndexMap] = d_5_8_672params(codewordLength);
       else % 13/16
            [infoLen, offsetWeight, columnIndexMap] = d_13_16_672params(codewordLength);
       end
end

% Parse optional input arguments
[outWholeCodeword, softDecision] = parseInputs(varargin{:});
earlyTermination = int8(earlyTermination);
compFinalParityChecks = int8(nargout == 3);

infoLen = round(infoLen);

% Initialize outputs
numParityBits = codewordLength - infoLen;
x_out = coder.nullcopy(zeros(codewordLength, numCodewords));
finalParityChecks = coder.nullcopy(zeros(numParityBits, numCodewords));
actualNumIter = coder.nullcopy(zeros(1, numCodewords));

% Decode
if isempty(coder.target) % Simulation
    y = zeros(codewordLength, numCodewords);

    for cwIndx = 1:numCodewords
        % C-Mex call, only doubles support
        [y(:,cwIndx), actualNumIter(cwIndx), finalParityChecks(:,cwIndx)] = ...
            mwcomm_ldpcdecode_mt(double(x(:,cwIndx)), maxNumIter, ...
            codewordLength, numParityBits, length(columnIndexMap)/2, Z, ...
            offsetWeight, columnIndexMap, int8(~softDecision), earlyTermination, ...
            compFinalParityChecks, algChoice, alphaBeta, ...
            int8(1)); % Use multi-threading
    end

    % Format output
    if softDecision
        if ~outWholeCodeword
            y = y(1:infoLen, :);
        end
    else
        if outWholeCodeword
            y = int8(y);
        else
            y = int8(y(1:infoLen, :));
        end
    end
else % codegen
    % Initialize output to fix dimension and data type
    if softDecision
        if outWholeCodeword
             y = coder.nullcopy(zeros(codewordLength, numCodewords));
        else
            y = coder.nullcopy(zeros(infoLen, numCodewords));
        end
    else
        if outWholeCodeword
            y = coder.nullcopy(zeros(codewordLength, numCodewords, 'int8'));
        else
            y = coder.nullcopy(zeros(infoLen, numCodewords, 'int8'));
        end
    end

    switch algChoice
        case 0 % Use BP
            for cwIndx = 1:numCodewords
                [x_out(:,cwIndx), actualNumIter(cwIndx), ...
                    finalParityChecks(:,cwIndx)] = ...
                    comm.internal.ldpc.BPDecode(x(:,cwIndx), maxNumIter, ...
                    codewordLength, numParityBits, length(columnIndexMap)/2, ...
                    offsetWeight, columnIndexMap, earlyTermination, compFinalParityChecks);
            end
        case 1 % Use Layered BP
            for cwIndx = 1:numCodewords
                [x_out(:,cwIndx), actualNumIter(cwIndx), ...
                    finalParityChecks(:,cwIndx)] = ...
                    comm.internal.ldpc.LayeredBPDecode(x(:,cwIndx), ...
                    maxNumIter, numParityBits, length(columnIndexMap)/2, ...
                    offsetWeight, columnIndexMap, earlyTermination, compFinalParityChecks);
            end
        case 2 % Use Layered normalized min-sum
            for cwIndx = 1:numCodewords
                [x_out(:,cwIndx), actualNumIter(cwIndx), ...
                    finalParityChecks(:,cwIndx)] = ...
                    comm.internal.ldpc.LayeredBPNormMSDecode(x(:,cwIndx), ...
                    maxNumIter, numParityBits, length(columnIndexMap)/2, ...
                    offsetWeight, columnIndexMap, earlyTermination, compFinalParityChecks, alphaBeta);
            end
        otherwise  % alg==3
            % Use Layered offset min-sum
            for cwIndx = 1:numCodewords
                [x_out(:,cwIndx), actualNumIter(cwIndx), ...
                    finalParityChecks(:,cwIndx)] = ...
                    comm.internal.ldpc.LayeredBPOffsetMSDecode(x(:,cwIndx), ...
                    maxNumIter, numParityBits, length(columnIndexMap)/2, ...
                    offsetWeight, columnIndexMap, earlyTermination, compFinalParityChecks, alphaBeta);
            end
    end

    % Format output
    if softDecision
        if outWholeCodeword
            y(:) = x_out;
        else
            y(:) = x_out(1:infoLen, :);
        end
    else
        if outWholeCodeword
            y(:) = int8(x_out <= 0);
        else
            y(:) = int8(x_out(1:infoLen, :) <= 0);
        end
    end
end
end


function d = ldpcinit(H)
    [d.offsetWeight,d.columnIndexMap] = ...
        comm.internal.ldpc.getDecoderParameters(H, false);
end

function [offsetWeight,columnIndexMap] = getldpcparams(d)
    offsetWeight = d.offsetWeight;
    columnIndexMap = d.columnIndexMap;
end

function [outWholeCodeword, softDecision] = parseInputs(varargin)
    outWholeCodeword = false;
    softDecision = false;

    for i = 1:(nargin)
        if strcmpi(varargin{i}, 'whole codeword')
            outWholeCodeword = true;
        elseif strcmpi(varargin{i}, 'soft decision')
            softDecision = true;
        end
    end
end

% Persistent variables for pre-computed data structures for LDPC codes When
% an LDPC code is used for the first time, load the corresponding
% pre-computed data structure and save it in a persistent variable.

%% Block length 648
function [infoLen, offsetWeight, columnIndexMap] = d_1_2_648params(codewordLength)
    persistent d_1_2_648
    infoLen = codewordLength * 1/2;
    if isempty(d_1_2_648)
        H = wlan.internal.ldpcMatrix('1/2', codewordLength);
        d_1_2_648 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_1_2_648);
end

function [infoLen, offsetWeight, columnIndexMap] = d_2_3_648params(codewordLength)
    persistent d_2_3_648
    infoLen = codewordLength * 2/3;
    if isempty(d_2_3_648)
        H = wlan.internal.ldpcMatrix('2/3', codewordLength);
        d_2_3_648 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_2_3_648);
end

function [infoLen, offsetWeight, columnIndexMap] = d_3_4_648params(codewordLength)
    persistent d_3_4_648
    infoLen = codewordLength * 3/4;
    if isempty(d_3_4_648)
        H = wlan.internal.ldpcMatrix('3/4', codewordLength);
        d_3_4_648 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_3_4_648);
end

function [infoLen, offsetWeight, columnIndexMap] = d_5_6_648params(codewordLength)   
    persistent d_5_6_648
    infoLen = codewordLength * 5/6;
    if isempty(d_5_6_648)
        H = wlan.internal.ldpcMatrix('5/6', codewordLength);
        d_5_6_648 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_5_6_648);  
end

%% Block length 1296
function [infoLen, offsetWeight, columnIndexMap] = d_1_2_1296params(codewordLength)
    persistent d_1_2_1296
    infoLen = codewordLength * 1/2;
    if isempty(d_1_2_1296)
        H = wlan.internal.ldpcMatrix('1/2', codewordLength);
        d_1_2_1296 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_1_2_1296);
end

function [infoLen, offsetWeight, columnIndexMap] = d_2_3_1296params(codewordLength)
    persistent d_2_3_1296           
    infoLen = codewordLength * 2/3;
    if isempty(d_2_3_1296)
        H = wlan.internal.ldpcMatrix('2/3', codewordLength);
        d_2_3_1296 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_2_3_1296);
end

function [infoLen, offsetWeight, columnIndexMap] = d_3_4_1296params(codewordLength)
    persistent d_3_4_1296           
    infoLen = codewordLength * 3/4;
    if isempty(d_3_4_1296)
        H = wlan.internal.ldpcMatrix('3/4', codewordLength);
        d_3_4_1296 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_3_4_1296); 
end

function [infoLen, offsetWeight, columnIndexMap] = d_5_6_1296params(codewordLength)
    persistent d_5_6_1296          
    infoLen = codewordLength * 5/6;
    if isempty(d_5_6_1296)
        H = wlan.internal.ldpcMatrix('5/6', codewordLength);
        d_5_6_1296 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_5_6_1296);
end

%% Block length 1944
function [infoLen, offsetWeight, columnIndexMap] = d_1_2_1944params(codewordLength)
    persistent d_1_2_1944 
    infoLen = codewordLength * 1/2;
    if isempty(d_1_2_1944)
        H = wlan.internal.ldpcMatrix('1/2', codewordLength); 
        d_1_2_1944 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_1_2_1944);
end

function [infoLen, offsetWeight, columnIndexMap] = d_2_3_1944params(codewordLength)
    persistent d_2_3_1944
    infoLen = codewordLength * 2/3;
    if isempty(d_2_3_1944)
        H = wlan.internal.ldpcMatrix('2/3', codewordLength); 
        d_2_3_1944 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_2_3_1944);  
end

function [infoLen, offsetWeight, columnIndexMap] = d_3_4_1944params(codewordLength)
    persistent d_3_4_1944
    infoLen = codewordLength * 3/4;
    if isempty(d_3_4_1944)
        H = wlan.internal.ldpcMatrix('3/4', codewordLength);
        d_3_4_1944 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_3_4_1944);
end

function [infoLen, offsetWeight, columnIndexMap] = d_5_6_1944params(codewordLength)
    persistent d_5_6_1944
    infoLen = codewordLength * 5/6;
    if isempty(d_5_6_1944)
        H = wlan.internal.ldpcMatrix('5/6', codewordLength);
        d_5_6_1944 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_5_6_1944);
end

%% Block length 672   
function [infoLen, offsetWeight, columnIndexMap] = d_1_2_672params(codewordLength)
    persistent d_1_2_672 
    infoLen = codewordLength * 1/2;
    if isempty(d_1_2_672)
        H = wlan.internal.ldpcMatrix('1/2', codewordLength);
        d_1_2_672 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_1_2_672);
end

function [infoLen, offsetWeight, columnIndexMap] = d_3_4_672params(codewordLength)
    persistent d_3_4_672 
    infoLen = codewordLength * 3/4;
    if isempty(d_3_4_672)
        H = wlan.internal.ldpcMatrix('3/4',  codewordLength);
        d_3_4_672 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_3_4_672);
end

function [infoLen, offsetWeight, columnIndexMap] = d_5_8_672params(codewordLength)
    persistent d_5_8_672 
    infoLen = codewordLength * 5/8;
    if isempty(d_5_8_672)
        H = wlan.internal.ldpcMatrix('5/8', codewordLength);
        d_5_8_672 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_5_8_672);
end

function [infoLen, offsetWeight, columnIndexMap] = d_13_16_672params(codewordLength)
    persistent d_13_16_672 
    infoLen = codewordLength * 13/16;
    if isempty(d_13_16_672)
        H = wlan.internal.ldpcMatrix('13/16', codewordLength);
        d_13_16_672 = ldpcinit(H);
    end
    [offsetWeight, columnIndexMap] = getldpcparams(d_13_16_672);
end
