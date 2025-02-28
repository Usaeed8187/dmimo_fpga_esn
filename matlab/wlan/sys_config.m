function  cfg = sys_config(mode, psdulen, modtype, cctype)
% Return system configuration
% 
% mode 1: 2Tx/2Rx, 2-stream, QPSK, r=1/2 BCC/LDPC
% mode 2: 4Tx/4Rx, 4-stream, QPSK, r=1/2 BCC/LDPC
%
% By Donald Liang, last updated Feb 3, 2025

% Common system params
cfg = struct('Nfft', 64, ...  % FFT size
             'Ncp', 16, ...   % Cyclic prefix size
             'Nsc', 56, ...   % Total number of valid subcarriers
             'Nsd', 52, ...   % Number of subcarriers for data
             'Npt', 4);       % Number of pilot subcarriers

% Channel coding: BCC or LDPC
if nargin < 4
    cfg.cctype = 'BCC';
elseif strcmpi(cctype, 'LDPC')
    cfg.cctype = 'LDPC';
else
    cfg.cctype = 'BCC';
end

% Modulation (bits per symbol)
if nargin < 3
    cfg.Mord = 2;  % default QPSK
elseif strcmpi(modtype,'QPSK')
    cfg.Mord = 2;
elseif strcmpi(modtype, '16QAM')
    cfg.Mord = 4;
elseif strcmpi(modtype, '64QAM')
    cfg.Mord = 6;
elseif strcmpi(modtype, '256QAM')
    cfg.Mord = 8;
elseif (modtype == 2 || modtype == 4 || modtype == 6 || modtype == 8)
    cfg.Mord = modtype;
else
    error('Unkown modulation')
end

cfg.spatialExpansion = false;
if strcmpi(mode, '1x1')
    cfg.Nt = 1;  % Number of transmit antennas
    cfg.Nr = 1;  % Number of receive antennas
    cfg.Nss = 1; % Number of spatial streams
elseif strcmpi(mode, '2t2s')
    cfg.Nt = 2;  % Number of transmit antennas
    cfg.Nr = 2;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
elseif strcmpi(mode, '2x4')
    cfg.Nt = 2;  % Number of transmit antennas
    cfg.Nr = 4;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
elseif strcmpi(mode, '4x2')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 2;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.spatialExpansion = true;
    % cfg.spatialMapping = sqrt(2)*spatialExpansionMatrix();
    cfg.spatialMapping = repmat([1,0; 0,1; 1,0; 0,1].', 1, 1, 56);   
elseif strcmpi(mode, '4x2csd')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 2;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.spatialExpansion = true;  % Enable spatial expansion
    cfg.spatialMapping = sqrt(2)*spatialExpansionMatrix();
elseif strcmpi(mode, '4x4')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 4;  % Number of receive antennas
    cfg.Nss = 4; % Number of spatial streams
else
    error('Unsupported mode')
end

% OFDM symbol length
cfg.SymLen = cfg.Nfft + cfg.Ncp;

% Coded bits per symbol
cfg.PSDUlen = psdulen;
bitspersym = cfg.Nss*cfg.Mord*cfg.Nsd;

% framing and BCC/LDPC encoder params
if strcmpi(cfg.cctype,'BCC')
    cfg.rate = 0.5;     % Fixed BCC rate
    sbits = 16;     % service bits (all zeros)
    tailbits = 6;   % tail padding bits per encoder
    % coded data bits per packet/frame
    databits = (8*cfg.PSDUlen + sbits + tailbits)/cfg.rate;
    % number of OFDM symbols
    cfg.Nsyms = ceil(databits/bitspersym);
    % number of padding bits
    cfg.PadBits = cfg.rate*(cfg.Nsyms*bitspersym - databits);
    cfg.encDataLen = cfg.Nsyms*bitspersym;
else
    cfg.rate = 0.5;     % Fixed LDPC rate
    cfg.ldpcLen = 1944; % LDPC block length
    sbits = 16;     % service bits (all zeros)
    % coded data bits per packet/frame
    databits = (8*cfg.PSDUlen + sbits)/cfg.rate;
    % number of OFDM symbols
    cfg.Nsyms = ceil(databits/bitspersym);
    numAvBits = bitspersym * cfg.Nsyms;
    % number of LDPC blocks
    cfg.numCW = ceil(databits/cfg.ldpcLen);
    cfg.numSHRT = max(0, cfg.numCW*cfg.ldpcLen*cfg.rate - databits*cfg.rate);
    cfg.numPuncBits = max(0,(cfg.numCW*cfg.ldpcLen)-numAvBits-cfg.numSHRT);
    if(((cfg.numPuncBits > 0.1*cfg.numCW*cfg.ldpcLen*(1-cfg.rate)) && ...
       (cfg.numSHRT < 1.2*cfg.numPuncBits*(cfg.rate/(1-cfg.rate))))|| ...
       (cfg.numPuncBits > 0.3*cfg.numCW*cfg.ldpcLen*(1-cfg.rate)))
       numAvBits = numAvBits + bitspersym; 
    end
    cfg.Nsyms = numAvBits/bitspersym;
    cfg.numPuncBits = max(0,(cfg.numCW*cfg.ldpcLen)-numAvBits-cfg.numSHRT);
    if cfg.numPuncBits > 0
        cfg.numRepBits = 0;
    else
        cfg.numRepBits = max(0, round(numAvBits-cfg.numCW*cfg.ldpcLen*(1-cfg.rate)) ...
                - databits*cfg.rate);
    end
    cfg.encDataLen = numAvBits;
    cfg.numPadding = 0;
    cfg.random_intlv = true;
    cfg.intlv = randperm(cfg.encDataLen);
end

% length of LTFs
cfg.LTFlen = cfg.Nss*(cfg.Nfft+cfg.Ncp);

% subcarrier and data/pilot indices for FFT block
cfg.scInd = [5:32 34:61];
cfg.dataInd = [5:11 13:25 27:32 34:39 41:53 55:61];
cfg.pilotInd = [12,26,40,54];

% data indices and pilot indices for SCDATA block
cfg.scdIdx = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 27, 28, 29, ...
    30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 56];
cfg.scpIdx = [8, 22, 35, 49];  % after Vsc/DC removed

