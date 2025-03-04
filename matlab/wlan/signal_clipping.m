function x = signal_clipping(x)

clipping = false;

idx = find(abs(real(x)) > 1.0);
if ~isempty(idx)
    clipping = true;
    x(idx) = x(idx)./abs(real(x(idx)));
end

idx = find(abs(imag(x)) > 1.0);
if ~isempty(idx)
    clipping = true;
    x(idx) = x(idx)./abs(real(x(idx)));
end

if clipping
    fprintf('Signal clipping happens');
end

