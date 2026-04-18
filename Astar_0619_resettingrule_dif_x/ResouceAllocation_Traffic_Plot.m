function ResouceAllocation_Traffic_Plot(NODES, resultNodes, cfg)
% ResouceAllocation_Traffic_Plot  Plot system and space usage along optimal path.
%
%   NODES       — full node cell array from A* run
%   resultNodes — optimal path indices (from Main.m)
%   cfg         — parameter struct from Main.m

N  = cfg.N;
M  = cfg.M;
S  = cfg.S;
C  = cfg.C;
Ni = cfg.Ni;

% ── Reconstruct remaining times for all nodes ────────────────────────────
for i = 1:length(NODES)
    node = NODES{i};
    for n = 1:N
        if node{2}(1, n) == 0
            node{3}(:, n) = C(:, n);
        end
    end
    NODES{i} = node;
end

% ── Extract data along optimal path ──────────────────────────────────────
num_points      = length(resultNodes);
t_w             = zeros(1, num_points);
route_usage     = zeros(N, num_points);
resc_occupation = zeros(N, num_points);
space_usage     = zeros(M, num_points);
generation_times = cell(N, 1);

% Actual arrival times (gamma - total traversal duration)
gamma_last = NODES{resultNodes(end)}{11};
alpha = cell(1, N);
for n = 1:N
    for i = 1:Ni(n)
        alpha{n}(i) = gamma_last{n}(i) - sum(C(:, n));
    end
end
xAlloc = NODES{resultNodes(end)}{15};

for idx = 1:num_points
    cur  = NODES{resultNodes(idx)};
    t_w(idx) = cur{5};

    if idx < num_points
        nxt = NODES{resultNodes(idx + 1)};
        V_next_whole = nxt{9};

        for n = 1:N
            d         = cur{2}(n);
            d_next    = nxt{2}(n);
            remain    = cur{3}(:, n);
            remain_next = nxt{3}(:, n);
            V_next    = nxt{9}(n, :);

            if abs(d) < 1e-5 && d_next < 100
                if any(V_next > 0)
                    route_usage(n, idx)     = 1;
                    resc_occupation(n, idx) = find(V_next > 0, 1);
                else
                    route_usage(n, idx)     = 0.5;
                    resc_occupation(n, idx) = 0;
                end
                generation_times{n} = [generation_times{n}, cur{5}];

            elseif abs(d) > 1e-5 && any(V_next > 0)
                route_usage(n, idx)     = 1;
                resc_occupation(n, idx) = find(V_next > 0, 1);

            elseif abs(d) > 1e-5 && abs(d_next) < 1e-5
                if sum(remain) > 1e-5
                    route_usage(n, idx) = 0.5;
                else
                    route_usage(n, idx) = 0;
                end

            elseif abs(d) > 1e-3 && abs(d_next) > 1e-3 && abs(d) < 100 && ...
                   ((sum(remain) < sum(remain_next) && all(V_next == 0)) || ...
                    (sum(remain) == sum(remain_next) && abs(sum(remain)) > 1e-3))
                route_usage(n, idx) = 0.5;

            else
                route_usage(n, idx) = 0;
            end
        end

        for m = 1:M
            if any(V_next_whole(:, m) > 0)
                space_usage(m, idx) = 1;
            end
        end
    end
end

% ── Plot ─────────────────────────────────────────────────────────────────
colors = {[0 1 0], [1 0.85 0], [1 0.5 0], [0 0.7 0.9], [1 0 0], [1 1 1]};

active_cols = any(route_usage > 0, 1) | any(space_usage > 0, 1);
active_t    = t_w(active_cols);
if ~isempty(active_t)
    t_1 = active_t(1)   - 0.5;
    t_f = active_t(end) + 0.5;
else
    t_1 = t_w(1);  t_f = t_w(end);
end

figure('Name', sprintf('Resource Allocation  (useWeakRule=%d)', cfg.useWeakRule));

% ── System usage subplots ─────────────────────────────────────────────────
for n = 1:N
    subplot(N + M, 1, n);
    set(gca, 'FontSize', 14);  hold on;

    for idx = 1:num_points - 1
        x_s = t_w(idx);  x_e = t_w(idx + 1);
        y   = route_usage(n, idx);
        if y > 0.7
            ci = resc_occupation(n, idx);
            fill([x_s x_e x_e x_s], [0 0 y y], colors{ci}, 'EdgeColor', 'none');
        end
    end

    stairs(t_w, route_usage(n, :), 'LineWidth', 2, 'Color', [0.53 0.81 0.92]);

    for gt = generation_times{n}
        xline(gt, 'b-.', 'LineWidth', 1.5);
    end
    for at = alpha{n}
        xline(at, '-.', 'LineWidth', 1.5, 'Color', [1 0 0]);
    end

    ylabel(sprintf('Sys%d (R%d)', n, cfg.routeAssignment(n)), ...
           'FontSize', 13, 'Rotation', 0, 'HorizontalAlignment', 'right');
    xlim([t_1 t_f]);  ylim([0 1.1]);  grid on;
end

% ── Space usage subplots ──────────────────────────────────────────────────
for m = 1:M
    subplot(N + M, 1, m + N);
    set(gca, 'FontSize', 14);  hold on;

    for idx = 1:num_points - 1
        x_s = t_w(idx);  x_e = t_w(idx + 1);
        if space_usage(m, idx) > 0
            fill([x_s x_e x_e x_s], [0 0 1 1], colors{m}, 'EdgeColor', 'none');
        end
    end

    stairs(t_w, space_usage(m, :), 'LineWidth', 2, 'Color', [0.53 0.81 0.92]);
    ylabel(sprintf('Sp%d', m), 'FontSize', 13, 'Rotation', 0, 'HorizontalAlignment', 'right');
    xlim([t_1 t_f]);  ylim([0 1.2]);  grid on;
end

xlabel('Time (s)', 'FontSize', 14);
sgtitle(sprintf('useWeakRule = %d  |  N = %d systems  |  routes = [%s]', ...
    cfg.useWeakRule, cfg.N, num2str(cfg.routeAssignment)), 'FontSize', 13);

% ── Resetting-rule rectangles (last occurrence per subtask-space pair) ────
for n = 1:N
    if isempty(xAlloc{n}),  continue;  end
    for k = 1:length(xAlloc{n})
        xi = xAlloc{n}{k};
        if isempty(xi),  continue;  end

        seen = containers.Map();
        for row = size(xi, 1):-1:1
            key = sprintf('%d-%d', xi{row,3}, xi{row,4});
            if ~isKey(seen, key),  seen(key) = row;  end
        end

        for key = keys(seen)
            row   = seen(key{1});
            t_s   = xi{row, 1};  t_e = xi{row, 2};
            m_idx = xi{row, 4};
            if t_s == t_e,  continue;  end
            col = colors{m_idx};

            subplot(N + M, 1, n);       hold on;
            fill([t_s t_e t_e t_s], [0 0 1 1], col, ...
                 'EdgeColor', 'k', 'LineWidth', 1.2, 'FaceAlpha', 0.7);

            subplot(N + M, 1, N + m_idx);  hold on;
            fill([t_s t_e t_e t_s], [0 0 1 1], col, ...
                 'EdgeColor', 'k', 'LineWidth', 1.2, 'FaceAlpha', 0.7);
        end
    end
end

end
