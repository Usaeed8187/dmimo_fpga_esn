function [ltf_precoded_time_domain] = cal_ltf_precoded_time_domain(cfg)

ltfRef = [1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, ...
       1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, ...
       1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, ...
       1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,-1,-1 ].';

r1 = read_complex_binary(cfg.csi_dir);
h1 = reshape(r1, cfg.Nr, cfg.Nt, cfg.Nsc, []);
if cfg.wideband
    h_current = mean(h1, [3,4]);
    
    [~, ~, V] = svd(h_current);
    precoding_vector = V(:,1);
    
    % precoding_vector = precoding_vector / mean(abs(precoding_vector).^2);
    % precoding_vector = sign(precoding_vector(1,1)) .* precoding_vector;
    
    ltfRef_precoded = zeros([size(ltfRef, 1) cfg.Nt]);
    for sc_ind = 1:cfg.Nsc
        ltfRef_current = ltfRef(sc_ind, 1);
        ltfRef_precoded(sc_ind, :)= precoding_vector * ltfRef_current;
    end
    

else

    channel_idx = 1;
    ltfRef_precoded = zeros([size(ltfRef, 1) cfg.Nt]);
    for sc_ind = 1:cfg.Nsc
        h_current = squeeze(h1(:, :, sc_ind, channel_idx));
        
        if strcmpi(cfg.mode, '2x4svd') 
            [~, ~, V] = svd(h_current);
            precoding_vector = V(:,1);
            precoding_vector = sign(precoding_vector(1,1)) .* precoding_vector;
        end
    
        ltfRef_current = ltfRef(sc_ind, 1);
        ltfRef_precoded(sc_ind, :)= precoding_vector * ltfRef_current;
    
    end
end


ltf_precoded_time_domain = ofdm_mod(cfg, ltfRef_precoded, true);


end % EOF
