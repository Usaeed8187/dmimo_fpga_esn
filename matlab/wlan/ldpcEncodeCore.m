function y = ldpcEncodeCore(infoBits,rate)
%ldpcEncodeCore Generate LDPC parity matrices
%
%   Y = ldpcEncodeCore(INFOBITS,RATE) calculates the parity check bits for
%   the binary input data (infoBits) using a WLAN LDPC code at the
%   specified rate (RATE).
% 
%   The input INFOBITS is of size Ns-by-NCW where NS is the number of
%   information bit of a codeword and NCW is number of LDPC codewords.
%
%   RATE must be equal to '1/2', '2/3', '3/4', '5/6', '5/8' and '13/16'.
%   RATE and the number of rows in INFOBITS should correspond to one of the
%   four valid block lengths: 648, 672, 1296, or 1944.
%   
%   To obtain the codewords, concatenate infoBits and y
%   vertically, i.e. [infoBits;y].
%
%   See also ldpcDecodeCore, ldpcMatrix.

%   Copyright 2015-2016 The MathWorks, Inc.

%#codegen

% Persistent variables for pre-computed data structures for LDPC codes
persistent PT_1_2_648     % coding rate = 1/2, block length = 648
persistent PT_1_2_1296    % coding rate = 1/2, block length = 1296
persistent PT_1_2_1944    % coding rate = 1/2, block length = 1944
persistent PT_1_2_672     % coding rate = 1/2, block length = 672(802.11ad)
persistent PT_2_3_648     % coding rate = 2/3, block length = 648
persistent PT_2_3_1296    % coding rate = 2/3, block length = 1296
persistent PT_2_3_1944    % coding rate = 2/3, block length = 1944
persistent PT_3_4_648     % coding rate = 3/4, block length = 648
persistent PT_3_4_672     % coding rate = 3/4, block length = 672(802.11ad)
persistent PT_3_4_1296    % coding rate = 3/4, block length = 1296
persistent PT_3_4_1944    % coding rate = 3/4, block length = 1944
persistent PT_5_6_648     % coding rate = 5/6, block length = 648
persistent PT_5_6_1296    % coding rate = 5/6, block length = 1296
persistent PT_5_6_1944    % coding rate = 5/6, block length = 1944
persistent PT_5_8_672     % coding rate = 5/8, block length = 672(802.11ad)
persistent PT_13_16_672   % coding rate = 13/16, block length = 672(802.11ad)    

infoLen = size(infoBits,1);

% When an LDPC code is used for the first time,
% load the corresponding pre-computed data structure and save it in a
% persistent variable.

% Compute parity check bits by direct modulo-2 matrix product

if strcmp(rate,'1/2') || isequal(rate,1/2)
    blockLength = infoLen/(1/2);
    switch blockLength
        case 648
            if isempty(PT_1_2_648)
                G = coder.load('wlan/ldpcMatrices.mat','PT_1_2_648');
                PT_1_2_648 = double(reshape(de2bi(G.PT_1_2_648,16),324,324));            
            end
            y = int8(mod(PT_1_2_648*double(infoBits),2));
        case 1296
            if isempty(PT_1_2_1296)
                G = coder.load('wlan/ldpcMatrices.mat','PT_1_2_1296');
                PT_1_2_1296 = double(reshape(de2bi(G.PT_1_2_1296,16),648,648));  
            end
            y = int8(mod(PT_1_2_1296*double(infoBits),2));
        case 1944
            if isempty(PT_1_2_1944)
                G = coder.load('wlan/ldpcMatrices.mat','PT_1_2_1944');
                PT_1_2_1944 = double(reshape(de2bi(G.PT_1_2_1944,16),972,972)); 
            end
            y = int8(mod(PT_1_2_1944*double(infoBits),2));
        otherwise
            if isempty(PT_1_2_672)
                % Generate matrix
                [~,G] = wlan.internal.ldpcMatrix('1/2',672);
                sizeG = size(G);
                PT_1_2_672 = double(G(:,sizeG(1)+1:end)).';
            end
            y = int8(mod(PT_1_2_672*double(infoBits),2));
    end
elseif strcmp(rate, '2/3') || isequal(rate,2/3)
    blockLength = infoLen/(2/3);
    switch blockLength
        case 648
            if isempty(PT_2_3_648)
                G = coder.load('wlan/ldpcMatrices.mat','PT_2_3_648');
                PT_2_3_648 = double(reshape(de2bi(G.PT_2_3_648,16),216,432)); 
            end
            y = int8(mod(PT_2_3_648*double(infoBits),2));
        case 1296
            if isempty(PT_2_3_1296)
                G = coder.load('wlan/ldpcMatrices.mat','PT_2_3_1296');
                PT_2_3_1296 = double(reshape(de2bi(G.PT_2_3_1296,16),432,864)); 
            end
            y = int8(mod(PT_2_3_1296*double(infoBits),2));
        otherwise % case 1944
            if isempty(PT_2_3_1944)
                G = coder.load('wlan/ldpcMatrices.mat','PT_2_3_1944');
                PT_2_3_1944 = double(reshape(de2bi(G.PT_2_3_1944,16),648,1296)); 
            end
            y = int8(mod(PT_2_3_1944*double(infoBits),2));
    end
elseif (strcmp(rate, '3/4') || isequal(rate, 3/4))
    blockLength = infoLen/(3/4);
    switch blockLength
        case 648
            if isempty(PT_3_4_648) 
                G = coder.load('wlan/ldpcMatrices.mat','PT_3_4_648');
                PT_3_4_648 = double(reshape(de2bi(G.PT_3_4_648,12),162,486));
            end
            y = int8(mod(PT_3_4_648*double(infoBits),2));
        case 1296
            if isempty(PT_3_4_1296)
                G = coder.load('wlan/ldpcMatrices.mat','PT_3_4_1296');
                PT_3_4_1296 = double(reshape(de2bi(G.PT_3_4_1296,16),324,972));
            end
            y = int8(mod(PT_3_4_1296*double(infoBits),2));
        case 1944
            if isempty(PT_3_4_1944)
                G = coder.load('wlan/ldpcMatrices.mat','PT_3_4_1944');
                PT_3_4_1944 = double(reshape(de2bi(G.PT_3_4_1944,12),486,1458));
            end
            y = int8(mod(PT_3_4_1944*double(infoBits),2));
        otherwise
            if isempty(PT_3_4_672)
                % Generate matrix
                [~,G] = wlan.internal.ldpcMatrix('3/4', 672);
                sizeG = size(G);
                PT_3_4_672 = double(G(:,sizeG(1)+1:end)).';
            end
            y = int8(mod(PT_3_4_672*double(infoBits),2));  
    end
elseif (strcmp(rate,'5/6') || isequal(rate,5/6))
    blockLength = infoLen/(5/6);
    switch blockLength
        case 648
            if isempty(PT_5_6_648)
                G = coder.load('wlan/ldpcMatrices.mat','PT_5_6_648');
                PT_5_6_648 = double(reshape(de2bi(G.PT_5_6_648,16),108,540));
            end
            y = int8(mod(PT_5_6_648*double(infoBits),2));
        case 1296
            if isempty(PT_5_6_1296)
                G = coder.load('wlan/ldpcMatrices.mat','PT_5_6_1296');
                PT_5_6_1296 = double(reshape(de2bi(G.PT_5_6_1296,16),216,1080));
            end
            y = int8(mod(PT_5_6_1296*double(infoBits),2));
        otherwise % case 1944
            if isempty(PT_5_6_1944)
                G = coder.load('wlan/ldpcMatrices.mat','PT_5_6_1944');
                PT_5_6_1944 = double(reshape(de2bi(G.PT_5_6_1944,16),324,1620));
            end
            y = int8(mod(PT_5_6_1944*double(infoBits),2));
    end 
elseif (strcmp(rate, '5/8') || isequal(rate, 5/8))
    if isempty(PT_5_8_672)
        % Generate matrix
        [~,G] = wlan.internal.ldpcMatrix('5/8',672);
        sizeG = size(G);
        PT_5_8_672 = double(G(:,sizeG(1)+1:end)).';
    end
        y = int8(mod(PT_5_8_672*double(infoBits),2));  
else
    if isempty(PT_13_16_672)
        % Generate matrix
        [~,G] = wlan.internal.ldpcMatrix('13/16',672);
        sizeG = size(G);
        PT_13_16_672 = double(G(:,sizeG(1)+1:end)).';
    end
        y = int8(mod(PT_13_16_672*double(infoBits),2));         
end
