function y = bcc_decode(x)
% 802.11 BCC decoder (for rate=1/2)

trellis = poly2trellis(7, [133 171]);
rate = trellis.numInputSymbols/trellis.numOutputSymbols;
tback = 30;  % trace back length

y = zeros(round(size(x,1)*rate), size(x,2), 'like', x);
for k=1:size(x,2)
    %tmp = vitdec(x(:,k), trellis, tback, 'trunc', 'hard');
    tmp = vitdec(x(:,k), trellis, tback, 'trunc', 'unquant');
    y(:,k) = tmp(:);
end


end % EOF
