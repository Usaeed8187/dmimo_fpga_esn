function y = psdu_scramble(x, scrinit)

if nargin < 2
    lsfr = uint8([1; 0; 1; 1; 1; 0; 1]);
else
    lsfr = de2bi(scrinit, 7, 'left-msb').';
end

scrbuf = zeros(127,1,'int8');
for k=1:127
    scrbuf(k) = xor(lsfr(1), lsfr(4)); % x7 xor x4
    lsfr(1:6) = lsfr(2:end);  % left-shift
    lsfr(7) = scrbuf(k);  % update x1
end

scrseq = repmat(scrbuf, ceil(size(x,1)/127), 1);
y = cast(xor(x, scrseq(1:size(x,1))), class(x));

end
