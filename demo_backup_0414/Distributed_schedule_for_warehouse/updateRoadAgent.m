function [x_road, y_road] = updateRoadAgent(agent_i, entries, valid_systems, ...
            xi_prev, yi_prev, xi_prev_bar, yi_prev_bar, ai_x, ai_y, const)
% Closed-form KKT solution — replaces YALMIP/quadprog.
% Per system n the problem decouples:
%   min  rho1*(x-x_bar)^2 + a_x*x + rho1*(y-y_bar)^2 + a_y*y
%   s.t. y >= x + Dt,  x >= 0
% Three KKT cases handled analytically.

N = const.N; rho1 = const.rho1; Dt = const.Dt;
kn = 1;

x_road = zeros(N, 1);
y_road = zeros(N, 1);

for n = valid_systems
    if isempty(entries{n}), continue; end

    x_bar = xi_prev_bar{n}(kn);
    y_bar = yi_prev_bar{n}(kn);
    a_x   = ai_x{n}(kn);
    a_y   = ai_y{n}(kn);

    x_unc = x_bar - a_x / (2*rho1);
    y_unc = y_bar - a_y / (2*rho1);

    if x_unc >= 0 && y_unc >= x_unc + Dt
        % Case 1: both constraints inactive
        x_n = x_unc;
        y_n = y_unc;

    elseif x_unc >= 0
        % Case 2: y = x + Dt active, x >= 0 may bind
        x_n = (x_bar + y_bar - Dt)/2 - (a_x + a_y)/(4*rho1);
        if x_n < 0
            x_n = 0;
            y_n = Dt;
        else
            y_n = x_n + Dt;
        end

    else
        % Case 3: x = 0 active
        x_n = 0;
        y_n = max(y_unc, Dt);
    end

    x_road(n) = x_n;
    y_road(n) = y_n;
end
