function  cfg = sys_config_nr(mode, psdulen, modtype, cctype)
% Return system configuration
%
% By Donald Liang, last updated June 20, 2025

% NCBPSSHORT % Number of coded bits per symbol (short)
% NDBPSSHORT % Number of data bits per symbol (short)
% Ntail   % Number of tail bits
% R       % Rate
% NSS     % Number of spatial streams
% NBPSCS  % Number of bits per subcarrier
% NDBPS   % Number of data bits per symbol
% NCBPS   % Number of coded bits per symbols
% NSD     % Number of data carrying subcarriers

% Common system params
cfg = struct('Nfft', 256, ... % FFT size
             'Ncp', 64, ...   % Cyclic prefix size
             'Nsc', 140, ...  % Total number of valid subcarriers
             'Nsd', 132, ...  % Number of subcarriers for data
             'Npt', 8);       % Number of pilot subcarriers

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
cfg.mode = mode;
if strcmpi(mode, '1t1s')
    cfg.Nt  = 1; % Number of transmit antennas
    cfg.Nss = 1; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
elseif strcmpi(mode, '2t1s')
    cfg.Nt  = 1; % Number of transmit antennas
    cfg.Nss = 1; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
elseif strcmpi(mode, '2t2s')
    cfg.Nt  = 2; % Number of transmit antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
elseif strcmpi(mode, '2t2s_csi')
    cfg.Nt  = 2; % Number of transmit antennas
    cfg.Nss = 2; % Number of spatial streams
elseif strcmpi(mode, '2t1s_svd')
    cfg.Nt = 2;  % Number of transmit antennas
    cfg.Nss = 1; % Number of spatial streams per user
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
elseif strcmpi(mode, '4t2s')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
    cfg.spatialExpansion = true;
    cfg.spatialMapping = repmat([1,0; 0,1; 1,0; 0,1].', 1, 1, cfg.Nsc);   
elseif strcmpi(mode, '4t2s_csd')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
    cfg.spatialExpansion = true;  % Enable spatial expansion
    cfg.spatialMapping = sqrt(2) * ...
        spatialExpansionMatrix(cfg.Nfft, cfg.Nsc, cfg.Nt, cfg.Nss);
elseif strcmpi(mode, '4t4s')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nss = 4; % Number of spatial streams
    cfg.Nr = min(cfg.Nt, cfg.Nss); % mumber of receive antennas (default)
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
    cfg.numPuncBits = max(cfg.numSHRT,(cfg.numCW*cfg.ldpcLen)-numAvBits-cfg.numSHRT);
    cfg.numRepBits = 0; % FIXME repetition not supported
    cfg.numPadding = numAvBits - cfg.numCW * 1944 + cfg.numSHRT + cfg.numPuncBits;
    cfg.encDataLen = numAvBits - cfg.numPadding;
    % cfg.intlv = randperm(cfg.encDataLen);
    % LDPC mapping (interleaving)
    k = (0:cfg.Nsd-1).';
    md = 9; % mapping distance
    cfg.intlv = md.*mod(k,(cfg.Nsd/md)) + floor(k.*md/cfg.Nsd)+1;  
end

% length of LTFs
cfg.LTFlen = cfg.Nss*(cfg.Nfft+cfg.Ncp);

% subcarrier and data/pilot indices for FFT block
cfg.scInd = [58:127 131:200];
cfg.dataInd = [58:66 68:80 82:106 108:120 122:127 ...
               131:136 138:150 152:176 178:190 192:200];
cfg.pilotInd = [67 81 107 121 137 151 177 191];

% data indices and pilot indices for SCDATA block
cfg.scdIdx = [1:9 11:23 25:49 51:63 65:76 78:90 92:116 118:130 132:140];
cfg.scpIdx = [10,24,50,64,77,91,117,131];  % after Vsc/DC removed

