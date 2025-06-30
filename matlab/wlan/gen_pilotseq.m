function pilots = gen_pilotseq(Nsd, Nsym, Nss, symOffset)
% Generate pilot sequence for a frame
%
% Inputs:
%   Nsd        number of data subcarriers (HT/VHT/HE)
%   Nsym       number of OFDM symbols in a frame
%   Nss        number of data streams
%   symOffset  initial symbol offset
% Output:
%   pilots     pilots symbols for a frame
%
% By Donald Liang, last updated March 20, 2022

if nargin < 3
    Nss = 1;  % default single stream
end

if nargin < 4
    if (Nsd == 52 || Nsd == 132)
        symOffset = 3; % HT/VHT
    else
        symOffset = 4; % HE
    end
end

% base pilot for 2/4 streams
if Nsd == 52
    pilotMod = 4;
    if Nss == 1
        basePilot = [1  1  1 -1];
    elseif Nss == 2
	    basePilot = [1  1 -1 -1; 1 -1 -1  1];
    elseif Nss == 4
	    basePilot = [1 1 1 -1; 1 1 -1 1; 1 -1 1 1; -1 1 1 1];
    end
elseif (Nsd == 234 || Nsd == 132)
    pilotMod = 8;
    basePilot = [1 1 1 -1 -1 1 1 1];
    % HE use single stream pilots
    basePilot = repmat(basePilot, Nss, 1);
else
    error("Pilot mode not supported")
end

% pilot polarity sequence
pn = [1  1  1  1 -1 -1 -1 1 -1 -1 -1 -1  1  1 -1  1 -1 -1  1 1 -1  1  1 -1  1  1  1  1 ...
    1  1 -1  1  1  1 -1 1  1 -1 -1  1  1  1 -1  1 -1 -1 -1 1 -1  1 -1 -1  1 -1 -1  1 ...
    1  1  1  1 -1 -1  1 1 -1 -1  1 -1  1 -1  1  1 -1 -1 -1 1  1 -1 -1 -1 -1  1 -1 -1 ...
    1 -1  1  1  1  1 -1 1 -1  1 -1  1 -1 -1 -1 -1 -1  1 -1 1  1 -1  1 -1  1  1  1 -1 ...
    -1  1 -1 -1 -1  1  1 1 -1 -1 -1 -1 -1 -1 -1].';

% pns = comm.PNSequence('Polynomial', 'x7+x3+1', 'InitialConditions', [1 1 1 1 1 1 1], 'SamplesPerFrame', 127+7);
% px = pns();
% pn = (1-2*px(8:end));

pilots = zeros(length(basePilot), Nsym, Nss);

for k = 0:Nsym-1
    idx = mod(k + (0:pilotMod-1)', pilotMod) + 1;
    polarity = pn(mod(k + symOffset, 127) + 1);
    for si = 1:Nss
        pilots(:, k+1, si) = basePilot(si, idx) * polarity;
    end
end

