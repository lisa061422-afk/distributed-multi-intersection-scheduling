function [x9_new, delay_cost] = updateAgent9(x9_prev, x9_prev_bar, ...
    a9_x,const)

N = const.N; rho1 = const.rho1; deadline = const.deadline; weight = const.weight;
    % YALMIP variables
    x9 = sdpvar(N, 1);  

    % 初始化
    Objective = 0; objective = 0; penalty = 0;
    Constraints = [];

    for n = 1:N
        x_bar = x9_prev_bar{n}(1);
        a_x   = a9_x{n}(1);
        ddl_n = deadline{n}(1);  % deadline 是 double 类型
            
        % 构建项：consensus + dual + delay penalty
        objective = objective ...
            + rho1 * (x9(n, 1) - x_bar)^2 ...
            + a_x * x9(n, 1);  % terminal cost

        % ---- 迟到罚项 ----
        late_n  = max(x9(n, 1) - ddl_n, 0);
        penalty = penalty + weight * late_n;
        % ---- 总目标（用于求解）----
        Objective = objective + penalty;

        Constraints = [Constraints, x9(n, 1) >= 0];
 
    end

% YALMIP 求解
options = sdpsettings('solver', 'gurobi', 'verbose', 0);
diagnostics = optimize(Constraints, Objective, options);

% 若求解成功，则返回最优值；否则 fallback 为上次值
if diagnostics.problem ~= 0
    warning('YALMIP: updateAgent9 infeasible (terminal agent).');
    % Always return a numeric N×1 vector, never the raw cell (which would
    % cause MATLAB struct() to expand into a struct array in the caller).
    x9_fb = NaN(N, 1);
    for nf_ = 1:N
        if iscell(x9_prev) && numel(x9_prev) >= nf_
            v_ = x9_prev{nf_};
            if isnumeric(v_) && ~isempty(v_), x9_fb(nf_) = v_(1); end
        elseif isnumeric(x9_prev) && numel(x9_prev) >= nf_
            x9_fb(nf_) = x9_prev(nf_);
        end
    end
    x9_new = x9_fb;
    delay_cost = NaN;  % fallback
else
    x9_new = value(x9);
    % 构造统一尺寸的 ddl 矩阵（NaN 填充）
    ddl_mat = NaN(N, 1);
    for n = 1:N
        ddl_mat(n, 1:length(deadline{n})) = deadline{n};
    end
    % 计算 delay（x9_new 是 N×I 矩阵）
    dif_val = max(x9_new - ddl_mat, 0);
    delay_cost = sum(dif_val(:), 'omitnan');

end
