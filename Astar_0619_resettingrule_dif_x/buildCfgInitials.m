function cfg = buildCfgInitials(cfg)
% buildCfgInitials  Compute alpha_tilde and initial_position from cfg parameters.
%   Called by Main.m and runBatch.m before runAstar.

v_max_vec = cfg.v_max * ones(1, cfg.N);
alpha_tilde      = cell(1, cfg.N);
initial_position = zeros(cfg.N, max(cfg.Ni));

for n = 1:cfg.N
    alpha_tilde{n}(1)     = (cfg.detect_range/2 - cfg.W/2) / v_max_vec(n) + cfg.d1(n);
    initial_position(n,1) = -(cfg.detect_range/2 - cfg.W/2 + cfg.d1(n)*v_max_vec(n));
    for i = 2:cfg.Ni(n)
        alpha_tilde{n}(i)     = alpha_tilde{n}(i-1) + cfg.T{n}{i-1};
        initial_position(n,i) = initial_position(n,i-1) - cfg.T{n}{i-1}*v_max_vec(n);
    end
end

cfg.alpha_tilde      = alpha_tilde;
cfg.initial_position = initial_position;
end
