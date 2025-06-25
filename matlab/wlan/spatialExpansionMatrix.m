function Q = spatialExpansionMatrix(Nfft, numSD, NumTx, numSTS)

% Default system params
if nargin < 1
    NumTx = 4;
    numSTS = 2;
    Nfft = 64;
    numSD = 56;
end

% Calculate cyclic shift matrix
if numSD == 242
    nn = [-122:-2 2:122].';
else
    nn = [-28:-1 1:28].';
end

% nn = [5:11 13:25 27:32 34:39 41:53 55:61].' - 33;
csd = [0 -8 -4 -12].';
phShift = exp(-1i*2*pi*csd*nn'/Nfft).';
Mcsd = permute(phShift,[1 3 2]);

% Calculate D matrix (Std 802.11-2012 Section 20.3.11.11.2)
% a = sqrt(2)/2
% D = [a, 0; 0, a; a, 0; 0, a];
base = eye(numSTS);
D = zeros(NumTx,numSTS);
for d=1:NumTx
   D(d,:) = base(mod(d-1,numSTS)+1,:); 
end
D = sqrt(numSTS/NumTx)*D;  % normalize power

% Calculate spatial expansion matrix for occupied subcarriers
Q = repmat(Mcsd,1,numSTS,1).*repmat(permute(D,[3 2 1]),numSD,1,1);
Q = permute(Q, [2,3,1]);

end
