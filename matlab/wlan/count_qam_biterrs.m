function biterrs = count_qam_biterrs(yd, hardbits, modtype, nss)
% Calculate QAM hard-decison BER

% default for two streams
if nargin < 4
    nss = 2;
end

assert(size(hardbits,2)==nss, "Incorect number of streams for hard-decision bits")

% QAM symbols for all streams
yd = reshape(yd, [], nss);

% Hard-decision demapping
mord = 2;
if strcmpi(modtype, '16QAM')
    mord = 4;
elseif strcmpi(modtype, '64QAM')
    mord = 6;
end

dbits = zeros(size(hardbits,1), nss); 
for n=1:nss
    % hdec = sign([real(yd(:,n)) imag(yd(:,n))].'); % QPSK
    hdec = qam_demapping(yd(:,n), mord);
    dbits(:,n) = 2*(hdec(:)-0.5);
end
biterrs = nnz(dbits ~= hardbits);


end % EOF
