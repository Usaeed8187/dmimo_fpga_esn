function  cfg = sys_config_he(mode, psdulen, modtype, cctype)
% Return system configuration
% 
% mode 1: 2Tx/2Rx, 2-stream, QPSK, r=1/2 BCC/LDPC
% mode 2: 4Tx/4Rx, 4-stream, QPSK, r=1/2 BCC/LDPC
%
% By Donald Liang, last updated July 20, 2021

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
             'Nsc', 242, ...  % Total number of valid subcarriers
             'Nsd', 234, ...  % Number of subcarriers for data
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
elseif (modtype == 2 || modtype == 4 || modtype == 6)
    cfg.Mord = modtype;
else
    error('Unkown modulation')
end

cfg.spatialExpansion = false;
if strcmpi(mode, '2x2')
    cfg.Nt = 2;  % Number of transmit antennas
    cfg.Nr = 2;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
    if nargin < 2
        cfg.PSDUlen = 512;
    else
        cfg.PSDUlen = psdulen;
    end
    cfg.ltfLen = 160;
elseif strcmpi(mode, '4x2')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 2;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.spatialExpansion = true;
    % cfg.spatialMapping = sqrt(2)*spatialExpansionMatrix();
    cfg.spatialMapping = repmat([1,0; 0,1; 1,0; 0,1].', 1, 1, 56);
    if nargin < 2
        cfg.PSDUlen = 512;
    else
        cfg.PSDUlen = psdulen;
    end    
elseif strcmpi(mode, '4x2csd')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 4;  % Number of receive antennas
    cfg.Nss = 2; % Number of spatial streams
    cfg.spatialExpansion = true;  % Enable spatial expansion
    cfg.spatialMapping = sqrt(2)*spatialExpansionMatrix();
    if nargin < 2
        cfg.PSDUlen = 512;
    else
        cfg.PSDUlen = psdulen;
    end
elseif strcmpi(mode, '4x4')
    cfg.Nt = 4;  % Number of transmit antennas
    cfg.Nr = 4;  % Number of receive antennas
    cfg.Nss = 4; % Number of spatial streams
    if nargin < 2
        cfg.PSDUlen = 980;
    else
        cfg.PSDUlen = psdulen;
    end
else
    error('Unsupported mode')
end

% OFDM symbol length
cfg.SymLen = cfg.Nfft + cfg.Ncp;

% Coded bits per symbol
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
cfg.scInd = [7:127 131:251];
cfg.dataInd = [7:12 14:38 40:80 82:106 108:127 ...
               131:150 152:176 178:218 220:244 246:251];
cfg.pilotInd = [13 39 81 107 151 177 219 245];

% data indices and pilot indices for SCDATA block
cfg.scdIdx = [1:6 8:32 34:74 76:100 102:141 143:167 169:209 211:235 237:242];
cfg.scpIdx = [7,33,75,101,142,168,210,236];  % after Vsc/DC removed

