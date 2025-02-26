function [padbits] = genPadding(len, seed)

if (nargin < 2)
    seed = 73;
end

stream = RandStream('mt19937ar');
reset(stream,seed);
padbits = randi(stream,[0 1],len,1,'int8');

end

