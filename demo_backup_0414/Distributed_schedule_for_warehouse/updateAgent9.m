function [x9_new, delay_cost] = updateAgent9(x9_prev, x9_prev_bar, ...
    a9_x, const)
% Closed-form solution — replaces YALMIP/quadprog.
% Per system n the problem decouples:
%   min  rho1*(x - x_bar)^2 + a_x*x + weight*max(x - ddl, 0)
%   s.t. x >= 0
% Convex piecewise-quadratic: three KKT regions solved analytically.

N = const.N; rho1 = const.rho1; deadline = const.deadline; weight = const.weight;
kn = 1;

x9_new = zeros(N, 1);

for n = 1:N
    x_bar = x9_prev_bar{n}(1);
    a_x   = a9_x{n}(1);
    ddl_n = deadline{n}(1);

    % Minimizer ignoring late penalty (left region x <= ddl)
    x_A = x_bar - a_x / (2*rho1);
    % Minimizer including late penalty (right region x >= ddl)
    x_B = x_bar - (a_x + weight) / (2*rho1);

    if x_A <= ddl_n
        x_opt = x_A;          % optimal in left region
    elseif x_B >= ddl_n
        x_opt = x_B;          % optimal in right region
    else
        x_opt = ddl_n;        % kink is optimal
    end

    x9_new(n) = max(x_opt, 0);
end

% Compute delay cost
ddl_vec = zeros(N, 1);
for n = 1:N
    ddl_vec(n) = deadline{n}(1);
end
dif_val    = max(x9_new - ddl_vec, 0);
delay_cost = sum(dif_val, 'omitnan');
