function y = bcc_encode(x)
% 802.11 BCC encoder (for rate=1/2)

trellis = poly2trellis(7, [133 171]);
rate = trellis.numInputSymbols/trellis.numOutputSymbols;

y = zeros(round(size(x,1)/rate), size(x,2), 'like', x);
for k=1:size(x,2)
    tmp = convenc(x(:,k), trellis);
    y(:,k) = tmp(:);
end


end % EOF
