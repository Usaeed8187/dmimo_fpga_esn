function y = bcc_deinterleave(cfg, x)
% BCC deinterleaver

% interleaver params
blocksize = cfg.Mord * cfg.Nsd;  % one OFDM symbol block
pcol = 13;  % interleaver block width
prow = 4*cfg.Mord;  % interleaver block height
prot = 11 * [0; 2; 1; 3];  % interleaver rotations for streams

% data permutation index
pidx = transpose(reshape(1:blocksize, pcol, prow));

% interleaver output index
oidx = zeros(blocksize, cfg.Nss);
for ss=1:cfg.Nss
    oidx(:,ss) = mod((0:blocksize-1).' - prot(ss)*cfg.Mord, blocksize) + 1;
end

% adjust shape of deinterleaver inputs
x = reshape(x, blocksize, [], cfg.Nss);

% deinterleaving per spatial stream
data = reshape(x, blocksize, [], cfg.Nss);
y = zeros(size(data),'like',x);
for ss=1:cfg.Nss
     y(pidx(:), :, ss) = data(oidx(:,ss),:,ss);
end

% reshape
y = reshape(y, [], cfg.Nss);

end % EOF
