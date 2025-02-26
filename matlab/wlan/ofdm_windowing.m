function y = ofdm_windowing(y, cpLen, offset)
% Add windowing between OFDM symbols with fixed window size of 2

if nargin < 3
    offset = 5;  % first OFDM symbol for regular windowing
end

% add postfix
y(1,offset:end,:) = 0.5*(y(1,offset:end,:) + y(cpLen+1,offset-1:end-1,:));

end
