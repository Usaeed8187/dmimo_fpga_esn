function Y = getLDPCparameters(numDBPS,rate,mSTBC,numPLD,varargin) 
%GETLDPCPARAMETERS LDPC encode and decode parameters.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Y = getLDPCparameters(NUMDBPS, RATE, MSTBC, NUMPLD) return the LDPC
%   encoding and decoding parameters for the specified number of data bits
%   per symbol, rate, STBC and number of payload bits as specified in [1],
%   section 20.3.11.17.3. The NUMPLD includes the number of data and
%   service bits in HT format. In VHT format the NUMPLD includes the number
%   of data, service and padded bits.
%
%   Y = getLDPCparameters(..., NUMSYMBOL) return the LDPC encoding and
%   decoding parameters for the specified number of data bits per symbol,
%   rate, STBC and number of payload bits as specified in [1] [2], section
%   20.3.11.17.3 and section 22.3.10.5.4 respectively. The NYMSYMBOL
%   represents the maximum number of OFDM symbols resulted due to LDPC
%   puncturing of VHT codewords. This input is only valid for VHT format.
%
%   %   References:
%   [1] IEEE Std 802.11(TM)-2012 IEEE Standard for Information technology -
%   Telecommunications and information exchange between systems - Local and
%   metropolitan area networks - Specific requirements - Part 11: Wireless
%   LAN Medium Access Control (MAC) and Physical Layer (PHY)
%   Specifications.
%   [2] IEEE Std 802.11ac(TM)-2013 IEEE Standard for Information technology
%   - Telecommunications and information exchange between systems - Local
%   and metropolitan area networks - Specific requirements - Part 11:
%   Wireless LAN Medium Access Control (MAC) and Physical Layer (PHY)
%   Specifications - Amendment 4: Enhancements for Very High Throughput for
%   Operation in Bands below 6 GHz.
%
%   See also wlanLDPCEncode, wlanLDPCDecode.

%   Copyright 2016-2017 The MathWorks, Inc.

%#codegen

narginchk(4,5);

% A: Compute the number of available bits, section 20.3.1.7.5, IEEE Std
% 802.11-2012, Eq 20-36
numCBPS  = numDBPS/rate; 
numAvBits = numCBPS*mSTBC*ceil(numPLD/(mSTBC*numCBPS*rate));

% B:Compute integer number of LDPC codewords. Table 20-16, Section
% 20.3.11.7.5, IEEE Std 802.11-2012

if (numAvBits<= 648)
    numCW = 1; % Number of LDPC code words
    if (numAvBits >= (numPLD + 912*(1 - rate)))
        lengthLDPC = 1296;
    else
        lengthLDPC = 648;
    end
elseif(numAvBits <= 1296)
    numCW = 1;
    if(numAvBits >= (numPLD + 1464*(1 - rate)))
        lengthLDPC = 1944;
    else
        lengthLDPC = 1296;
    end
elseif(numAvBits <= 1944)
    numCW = 1;
    lengthLDPC = 1944;
elseif(numAvBits <= 2592)
    numCW = 2; 
    if(numAvBits >= (numPLD + 2916*(1 - rate)))
        lengthLDPC = 1944;
    else
        lengthLDPC = 1296;
    end
else
    numCW = ceil(numPLD/(1944*rate));
    lengthLDPC = 1944;
end

% C: Compute the number of shortening bits for HT and VHT, Eq 20-37, Eq
% 20-38
numSHRT = max(0, numCW*lengthLDPC*rate-numPLD);                 
numSPCW = floor(numSHRT/numCW);
shortBoundary = rem(numSHRT, numCW);
vecShortenBits = zeros(1, numCW);
vecShortenBits(1:shortBoundary)= numSPCW+1;
vecShortenBits(shortBoundary+1:end)= numSPCW;

% D:Compute the number of bits to be punctured for HT and VHT
numPunctureBits = max(0,(numCW*lengthLDPC)-numAvBits-numSHRT);

if nargin == 5
    numSymbol = varargin{1};
    % Only for VHT as defined in IEEE Std 802.11ac-2013, Section
    % 22.3.10.5.4, Eq 22-65. When NUMSYM in Eq 22-67 is known.
    numSymMaxInit = numPLD/numDBPS;                     
    if (numSymbol > numSymMaxInit)
        numAvBits = numAvBits + numCBPS*mSTBC;
    end
else
    % Compute the number of bits to be punctured both for HT and VHT, Eq
    % 20-39
    if(((numPunctureBits > 0.1*numCW*lengthLDPC*(1-rate)) && ...
       (numSHRT < 1.2*numPunctureBits*(rate/(1-rate))))|| ...
       (numPunctureBits > 0.3*numCW*lengthLDPC*(1-rate)))
       numAvBits = numAvBits + numCBPS*mSTBC; 
    end
    numSymbol = numAvBits/numCBPS;  
end

% Number of puncture bits, Eq 20-40
numPunctureBits = max(0,(numCW*lengthLDPC)-numAvBits-numSHRT);
vecPunctureBits = zeros(1, numCW);
numPPCW = floor(numPunctureBits/numCW); 
punctureBoundary = rem(numPunctureBits, numCW); 

if (numPunctureBits > 0)
   vecPunctureBits(1:punctureBoundary) = numPPCW+1;
   vecPunctureBits(punctureBoundary+1:numCW) = numPPCW;
   vecRepeatBits = zeros(1,numCW);
else  
   % E:Compute the number of coded bits to be repeated both for HT and VHT,
   % Eq 20-42
   numRep = max(0, round(numAvBits-numCW* ...
                   lengthLDPC*(1-rate))-numPLD);
   repeatBits = floor(numRep/numCW);
   extraBitLoc = rem(numRep, numCW);                          
   vecRepeatBits = repmat(repeatBits, 1, numCW);
   vecRepeatBits(1:extraBitLoc) = repeatBits+1; 
end

% Number of payload bits within a codeword
vecPayloadBits = lengthLDPC*rate-vecShortenBits;

% Set output
Y = struct( ...
        'VecPayloadBits',   vecPayloadBits, ...
        'Rate',             rate, ...
        'NumCBPS',          numCBPS, ...
        'NumCW',            numCW, ...
        'LengthLDPC',       lengthLDPC, ...
        'VecShortenBits',   vecShortenBits, ...
        'VecPunctureBits',  vecPunctureBits, ...
        'VecRepeatBits',    vecRepeatBits, ...
        'NumSymbol',        numSymbol, ...
        'NumAvailableBits', numAvBits);
        
end