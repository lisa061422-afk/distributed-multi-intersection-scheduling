function [x_road, y_road] = updateRoadAgent(agent_i, entries, valid_systems, ...
            xi_prev, yi_prev, xi_prev_bar, yi_prev_bar, ai_x, ai_y,const)

N = const.N; rho1 = const.rho1; Dt = const.Dt;
    % YALMIP variables
    x = sdpvar(N, 1);
    y = sdpvar(N, 1);

    Objective = 0; %initialization
    Constraints = [];
    
    kn = 1; 
    for n = valid_systems
        if isempty(entries{n}) continue; end
        x_bar = xi_prev_bar{n}(kn);
        y_bar = yi_prev_bar{n}(kn);
        a_xn  = ai_x{n}(kn);
        a_yn  = ai_y{n}(kn);

        % 构建目标函数
        Objective = Objective ...
            + rho1 * (x(n, kn) - x_bar)^2 + a_xn * x(n, kn) ...
            + rho1 * (y(n, kn) - y_bar)^2 + a_yn * y(n, kn);
        % 构建约束：y ≥ x + Dt, 非负约束
        Constraints = [Constraints, ...
            y(n, kn) >= x(n, kn) + Dt, ...
            x(n, kn) >= 0, ...
            y(n, kn) >= 0];
    end

% YALMIP 求解
options = sdpsettings('solver', 'quadprog', 'verbose', 0);
diagnostics = optimize(Constraints, Objective, options);

% 若求解成功，则返回最优值；否则 fallback 为上次值
if diagnostics.problem ~= 0
    warning(['YALMIP: updateRoadAgent infeasible for agent ', num2str(agent_i)]);
    x_road = xi_prev;
    y_road = yi_prev;
else
    x_road = value(x);
    y_road = value(y);
end