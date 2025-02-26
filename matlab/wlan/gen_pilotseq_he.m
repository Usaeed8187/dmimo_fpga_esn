function pilots = gen_pilotseq_he(nSym, Nss, symOffset)
%   PILOTS = hePilots(RUSIZE,NUMSTS,N,Z) returns the pilot sequence as per
%   IEEE P802.11ax/D4.1, Section 27.3.11.13.
%
%   RUSIZE is the RU size in subcarriers and must be one of 26, 52, 106,
%   242, 484, 996, or 1992.
%
%   NUMSTS is the number of space-time streams.
%
%   N is the OFDM symbol index.
%
%   Z is the OFDM symbol offset index.

if nargin < 3
    symOffset = 4;
end

% base pilot
pilotMod = 8;
basePilot = [1 1 1 -1 -1 1 1 1].';
% pilot polarity sequence
pn = [1  1  1  1 -1 -1 -1 1 -1 -1 -1 -1  1  1 -1  1 -1 -1  1 1 -1  1  1 -1  1  1  1  1 ...
    1  1 -1  1  1  1 -1 1  1 -1 -1  1  1  1 -1  1 -1 -1 -1 1 -1  1 -1 -1  1 -1 -1  1 ...
    1  1  1  1 -1 -1  1 1 -1 -1  1 -1  1 -1  1  1 -1 -1 -1 1  1 -1 -1 -1 -1  1 -1 -1 ...
    1 -1  1  1  1  1 -1 1 -1  1 -1  1 -1 -1 -1 -1 -1  1 -1 1  1 -1  1 -1  1  1  1 -1 ...
    -1  1 -1 -1 -1  1  1 1 -1 -1 -1 -1 -1 -1 -1].';

pilots = zeros(8, nSym, Nss);
for k = 0:nSym-1
    idx = mod(k + (0:pilotMod-1)', pilotMod) + 1;
    polarity = pn(mod(k + symOffset, 127) + 1);
    for si = 1:Nss % Same pilots for each STS
        pilots(:, k+1, si) = basePilot(idx) * polarity;
    end
end
