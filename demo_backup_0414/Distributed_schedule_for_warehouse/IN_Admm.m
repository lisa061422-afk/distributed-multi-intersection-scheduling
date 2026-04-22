function [best_x, best_y, best_alpha, best_gamma, best_idx, NODES] ...
            = IN_Admm(NODES,LEAF,agent_i, entries,...
                xi_prev, yi_prev, xi_prev_bar, yi_prev_bar, ai_x, ai_y,valid_systems, MapMat, Cmat,const)


rho1 = const.rho1; rho2 = const.rho2; N = const.N; alpha_tilde = const.alpha_tilde;
best_cost = Inf; best_x = []; best_y = [];

kn = 1; % per car per system, no recurring tasks

% FUTURE: this loop is a parfor candidate — each leaf's YALMIP solve is
% independent. Currently blocked by MATLAB nested-parfor restriction
% (intersections already run inside parfeval workers). See Idea A/B/C in
% INi_Admm_DecisionTree.m for the full parallelisation roadmap.
for i = 1:length(LEAF)
    idx = LEAF(i);
    gamma = NODES{idx}{11}; alpha = NODES{idx}{16};
    %-------------------solve yalmip---------------------------------
    % YALMIP variables
    x = sdpvar(N, 1); y = sdpvar(N, 1);

    Objective = 0; %initialization
    Constraints = [];

    %% 提取 alpha_vec 和 gamma_vec
    % alpha_vec = NaN(N, I); gamma_vec = NaN(N, I);
    for n = valid_systems % traverse all routes that pass agent i
        if isempty(entries{n}), continue; end
        if isempty(xi_prev_bar{n}) || isempty(alpha{n}) || isempty(gamma{n})
            fprintf('[IN_Admm] Agent %d sys %d leaf %d: empty bar/alpha/gamma — skipping system.\n', agent_i, n, idx);
            continue;
        end

        x_bar = xi_prev_bar{n}(kn);
        y_bar = yi_prev_bar{n}(kn);
        a_x   = ai_x{n}(kn);
        a_y   = ai_y{n}(kn);
        alpha_kn = alpha{n}(kn);
        gamma_kn = gamma{n}(kn);
        % 对 system n 的所有车辆（整行）计算 cost
        Objective = Objective ...
                + rho1 * (x(n, kn) - x_bar)^2 + a_x * x(n, kn) ...
                + rho1 * (y(n, kn) - y_bar)^2 + a_y * y(n, kn) ...
                + rho2 * (x(n, kn) - alpha_kn)^2 ...
                + rho2 * (y(n, kn) - gamma_kn)^2;
        % 基本约束：y = x + p  
        Constraints = [Constraints, ...
             y(n, kn) == x(n, kn) + (gamma_kn - alpha_kn), ...
             x(n, kn) >= alpha_tilde{n}(kn), ...
             y(n, kn) >= 0]; 
    end

    % % 保证 system n 的任务按顺序执行
    % for n = valid_systems
    %     kn_set = entries{n};
    %     for k = 2:length(kn_set)
    %         prev = kn_set(k-1);
    %         curr = kn_set(k);
    %         Constraints = [Constraints, x(n, curr) >= y(n, prev)];
    %     end
    % end
 
    %% 互斥约束（调用函数）
    Constraints = addMutualExclusion_SpaceLevel_Nonpreemptive( ...
        Constraints, MapMat, Cmat, x, y, valid_systems, gamma, 1e-4, agent_i);


    % 求解
    options = sdpsettings('solver', 'gurobi', 'verbose', 0);
    diagnostics = optimize(Constraints, Objective, options);

    if diagnostics.problem ~= 0
        warning('Infeasible or solver failed. Returning previous value.');
        x_opt = zeros(N, 1);
        y_opt = zeros(N, 1);
        for nn = valid_systems
            x_opt(nn) = xi_prev{nn}(kn);
            y_opt(nn) = yi_prev{nn}(kn);
        end
        cost = value(-1);
    else
        x_opt = value(x);
        y_opt = value(y);
        cost = value(Objective);
      
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