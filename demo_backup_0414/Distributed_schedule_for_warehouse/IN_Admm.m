function [best_x, best_y, best_alpha, best_gamma, best_idx, NODES] ...
            = IN_Admm(NODES,LEAF,agent_i, entries,...
                xi_prev, yi_prev, xi_prev_bar, yi_prev_bar, ai_x, ai_y,valid_systems, MapMat, Cmat,const)


rho1 = const.rho1; rho2 = const.rho2; N = const.N; alpha_tilde = const.alpha_tilde;
best_cost = Inf; best_x = []; best_y = [];

kn = 1; % per car per system, no recurring tasks

use_quadprog = isfield(const,'use_quadprog') && const.use_quadprog;

% ── quadprog pre-computation (shared across leaves) ──────────────────────
if use_quadprog
    A_coef  = 4*(rho1 + rho2);   % quadprog: min 0.5*x'Hx + f'x
    nv      = numel(valid_systems);
    idx_map = zeros(N, 1);
    idx_map(valid_systems) = 1:nv;
    qp_opts = optimset('Display', 'off');
end

for i = 1:length(LEAF)
    idx   = LEAF(i);
    gamma = NODES{idx}{11}; alpha = NODES{idx}{16};

    if use_quadprog
        % ── quadprog path ─────────────────────────────────────────────
        H_mat  = A_coef * eye(nv);
        f_vec  = zeros(nv, 1);
        lb_vec = -inf(nv, 1);
        C_vec  = zeros(N, 1);

        for ni = 1:nv
            n = valid_systems(ni);
            if isempty(entries{n}), continue; end
            if isempty(xi_prev_bar{n}) || isempty(alpha{n}) || isempty(gamma{n})
                fprintf('[IN_Admm] Agent %d sys %d leaf %d: empty bar/alpha/gamma — skipping system.\n', agent_i, n, idx);
                continue;
            end
            x_bar    = xi_prev_bar{n}(kn);
            y_bar    = yi_prev_bar{n}(kn);
            a_x      = ai_x{n}(kn);
            a_y      = ai_y{n}(kn);
            alpha_kn = alpha{n}(kn);
            gamma_kn = gamma{n}(kn);
            C_n      = gamma_kn - alpha_kn;
            C_vec(n) = C_n;
            f_vec(ni)  = a_x + a_y + 2*rho1*(C_n - x_bar - y_bar) - 4*rho2*alpha_kn;
            lb_vec(ni) = alpha_tilde{n}(kn);
        end

        [Aineq, bineq] = buildMutualExclusionConstraints_numeric(...
            MapMat, Cmat, valid_systems, gamma, idx_map, C_vec, 1e-4);

        [x_vec, ~, exitflag] = quadprog(H_mat, f_vec, Aineq, bineq, [], [], lb_vec, [], [], qp_opts);

        if exitflag <= 0
            x_opt = zeros(N, 1); y_opt = zeros(N, 1);
            for nn = valid_systems
                x_opt(nn) = xi_prev{nn}(kn); y_opt(nn) = yi_prev{nn}(kn);
            end
            cost = Inf;
        else
            x_opt = zeros(N, 1);
            for ni = 1:nv, x_opt(valid_systems(ni)) = x_vec(ni); end
            y_opt = x_opt + C_vec;

            if any(isnan(x_opt(valid_systems))) || any(isnan(y_opt(valid_systems)))
                fprintf('[IN_Admm] Agent %d leaf %d: quadprog NaN — fallback to xi_prev.\n', agent_i, idx);
                x_opt = zeros(N, 1); y_opt = zeros(N, 1);
                for nn = valid_systems
                    x_opt(nn) = xi_prev{nn}(kn); y_opt(nn) = yi_prev{nn}(kn);
                end
                cost = Inf;
            else
                cost = 0;
                for ni2 = 1:nv
                    n = valid_systems(ni2); xn = x_opt(n); yn = y_opt(n);
                    cost = cost ...
                        + rho1*(xn - xi_prev_bar{n}(kn))^2 + ai_x{n}(kn)*xn ...
                        + rho1*(yn - yi_prev_bar{n}(kn))^2 + ai_y{n}(kn)*yn ...
                        + rho2*(xn - alpha{n}(kn))^2 ...
                        + rho2*(yn - gamma{n}(kn))^2;
                end
            end
        end

    else
        % ── YALMIP + Gurobi path (original) ──────────────────────────
        x = sdpvar(N, 1);
        C_vec = zeros(N, 1);
        Objective = 0; Constraints = [];

        for n = valid_systems
            if isempty(entries{n}), continue; end
            if isempty(xi_prev_bar{n}) || isempty(alpha{n}) || isempty(gamma{n})
                fprintf('[IN_Admm] Agent %d sys %d leaf %d: empty bar/alpha/gamma — skipping system.\n', agent_i, n, idx);
                continue;
            end
            x_bar    = xi_prev_bar{n}(kn); y_bar = yi_prev_bar{n}(kn);
            a_x      = ai_x{n}(kn);        a_y   = ai_y{n}(kn);
            alpha_kn = alpha{n}(kn);        gamma_kn = gamma{n}(kn);
            C_n      = gamma_kn - alpha_kn; C_vec(n) = C_n;
            Objective = Objective ...
                + rho1*(x(n) - x_bar)^2       + a_x*x(n) ...
                + rho1*(x(n)+C_n - y_bar)^2   + a_y*(x(n)+C_n) ...
                + 2*rho2*(x(n) - alpha_kn)^2;
            Constraints = [Constraints, x(n) >= alpha_tilde{n}(kn)];
        end
        y_expr = x + C_vec;
        Constraints = addMutualExclusion_SpaceLevel_Nonpreemptive( ...
            Constraints, MapMat, Cmat, x, y_expr, valid_systems, gamma, 1e-4, agent_i);

        options = sdpsettings('solver', 'gurobi', 'verbose', 0);
        diagnostics = optimize(Constraints, Objective, options);

        if diagnostics.problem ~= 0
            warning('Infeasible or solver failed. Returning previous value.');
            x_opt = zeros(N, 1); y_opt = zeros(N, 1);
            for nn = valid_systems
                x_opt(nn) = xi_prev{nn}(kn); y_opt(nn) = yi_prev{nn}(kn);
            end
            cost = Inf;
        else
            x_opt = value(x); y_opt = x_opt + C_vec; cost = value(Objective);
            if any(isnan(x_opt(valid_systems))) || any(isnan(y_opt(valid_systems)))
                fprintf('[IN_Admm] Agent %d leaf %d: Gurobi NaN — fallback to xi_prev.\n', agent_i, idx);
                x_opt = zeros(N, 1); y_opt = zeros(N, 1);
                for nn = valid_systems
                    x_opt(nn) = xi_prev{nn}(kn); y_opt(nn) = yi_prev{nn}(kn);
                end
                cost = Inf;
            end
        end
    end

    if ~isnan(cost) && cost < best_cost
        best_cost = cost;
        best_x = x_opt;
        best_y = y_opt;
        best_gamma = gamma;
        best_alpha = alpha;
        best_idx = idx;
    end
end

% Fallback: if every leaf's solver failed, reuse previous x/y to avoid
% returning empty best_x/best_y (which crashes the merge step).
if isempty(best_x)
    fprintf('[IN_Admm] Agent %d: all %d leaves infeasible — fallback to previous x/y.\n', agent_i, length(LEAF));
    warning('IN_Admm: all %d leaves infeasible — using previous x/y as fallback.', length(LEAF));
    best_x = zeros(N, 1);
    best_y = zeros(N, 1);
    best_alpha = cell(1, N);
    best_gamma = cell(1, N);
    best_idx = LEAF(1);
    for nn = valid_systems
        best_x(nn) = xi_prev{nn}(1);
        best_y(nn) = yi_prev{nn}(1);
        best_alpha{nn} = NODES{best_idx}{16}{nn};
        best_gamma{nn} = NODES{best_idx}{11}{nn};
    end
end