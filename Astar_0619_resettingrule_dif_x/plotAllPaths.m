function plotAllPaths(NODES, LEAF, cfg, maxPaths)
% plotAllPaths  Compare resource allocation across all leaf paths.
%
%   NODES    — full node cell array from A* run
%   LEAF     — leaf node index array (from global LEAF in Main.m)
%   cfg      — parameter struct from Main.m
%   maxPaths — (optional) max columns to show, default 8

if nargin < 4,  maxPaths = 8;  end

N  = cfg.N;  M = cfg.M;  S = cfg.S;
C  = cfg.C;  Ni = cfg.Ni;

% ── Collect all leaf paths, sort by g-cost ────────────────────────────────
num_leaves = length(LEAF);
gcosts = arrayfun(@(li) NODES{li}{10}, LEAF);
[gcosts_sorted, sidx] = sort(gcosts, 'ascend');
LEAF_sorted = LEAF(sidx);

num_show = min(num_leaves, maxPaths);
LEAF_show  = LEAF_sorted(1:num_show);
gcosts_show = gcosts_sorted(1:num_show);

% ── Compute global time range for consistent x-axis ──────────────────────
t1_global = Inf;  tf_global = -Inf;
allPaths = cell(1, num_show);
for p = 1:num_show
    allPaths{p} = tracePath_(NODES, LEAF_show(p));
    tw_p = cellfun(@(ni) NODES{ni}{5}, num2cell(allPaths{p}));
    t1_global = min(t1_global, tw_p(1));
    tf_global = max(tf_global, tw_p(end));
end

% ── Build figure ─────────────────────────────────────────────────────────
nRows = N + M;
fig = figure('Name', sprintf('All Paths Comparison  (useWeakRule=%d)', cfg.useWeakRule));
tl  = tiledlayout(nRows, num_show, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('All leaf paths  |  useWeakRule=%d  |  routes=[%s]  |  sorted by g-cost', ...
    cfg.useWeakRule, num2str(cfg.routeAssignment)), 'FontSize', 11);

colors = {[0 1 0],[1 0.85 0],[1 0.5 0],[0 0.7 0.9],[1 0 0],[1 1 1]};

for p = 1:num_show
    pathNodes = allPaths{p};
    is_opt    = (p == 1);   % cheapest = leftmost column

    % ── Extract data for this path ────────────────────────────────────────
    [t_w, route_usage, resc_occ, space_usage, gen_times, alpha_arr, xAlloc] = ...
        extractPathData_(NODES, pathNodes, cfg);

    active_cols = any(route_usage > 0, 1) | any(space_usage > 0, 1);
    active_t    = t_w(active_cols);
    if ~isempty(active_t)
        t1 = active_t(1)   - 0.5;
        tf = active_t(end) + 0.5;
    else
        t1 = t1_global;  tf = tf_global;
    end

    % ── System rows ──────────────────────────────────────────────────────
    for n = 1:N
        tile_idx = (n-1)*num_show + p;
        ax = nexttile(tile_idx);
        hold(ax, 'on');

        for idx = 1:length(t_w)-1
            xs = t_w(idx);  xe = t_w(idx+1);
            yv = route_usage(n, idx);
            if yv > 0.7
                ci = resc_occ(n, idx);
                if ci >= 1 && ci <= length(colors)
                    fill(ax, [xs xe xe xs], [0 0 yv yv], colors{ci}, 'EdgeColor','none');
                end
            end
        end
        stairs(ax, t_w, route_usage(n,:), 'LineWidth', 1.2, 'Color', [0.4 0.7 0.9]);
        for gt = gen_times{n}
            xline(ax, gt, 'b-.', 'LineWidth', 1);
        end
        for at = alpha_arr{n}
            xline(ax, at, '-.', 'LineWidth', 1, 'Color', [1 0 0]);
        end

        xlim(ax, [t1 tf]);  ylim(ax, [0 1.2]);
        set(ax, 'FontSize', 8, 'XTickLabel', {});

        % Column header on first row
        if n == 1
            if is_opt
                title(ax, sprintf('\\bf Path %d*\ng=%.3f', p, gcosts_show(p)), ...
                    'FontSize', 8, 'Color', [0.8 0 0]);
            else
                title(ax, sprintf('Path %d\ng=%.3f', p, gcosts_show(p)), 'FontSize', 8);
            end
        end
        % Row label on first column
        if p == 1
            ylabel(ax, sprintf('Sy%d\n(R%d)', n, cfg.routeAssignment(n)), ...
                'FontSize', 7, 'Rotation', 0, 'HorizontalAlignment', 'right');
        else
            set(ax, 'YTickLabel', {});
        end
    end

    % ── Space rows ───────────────────────────────────────────────────────
    for m = 1:M
        tile_idx = (N + m - 1)*num_show + p;
        ax = nexttile(tile_idx);
        hold(ax, 'on');

        for idx = 1:length(t_w)-1
            xs = t_w(idx);  xe = t_w(idx+1);
            if space_usage(m, idx) > 0
                fill(ax, [xs xe xe xs], [0 0 1 1], colors{m}, 'EdgeColor','none');
            end
        end
        stairs(ax, t_w, space_usage(m,:), 'LineWidth', 1.2, 'Color', [0.4 0.7 0.9]);

        % Resetting-rule rectangles
        for n = 1:N
            if isempty(xAlloc{n}),  continue;  end
            for k = 1:length(xAlloc{n})
                xi = xAlloc{n}{k};
                if isempty(xi),  continue;  end
                seen = containers.Map();
                for row = size(xi,1):-1:1
                    key = sprintf('%d-%d', xi{row,3}, xi{row,4});
                    if ~isKey(seen, key),  seen(key) = row;  end
                end
                for key = keys(seen)
                    row = seen(key{1});
                    ts = xi{row,1};  te = xi{row,2};  mi = xi{row,4};
                    if ts == te || mi ~= m,  continue;  end
                    fill(ax, [ts te te ts], [0 0 1 1], colors{mi}, ...
                        'EdgeColor','k','LineWidth',0.8,'FaceAlpha',0.7);
                end
            end
        end

        xlim(ax, [t1 tf]);  ylim(ax, [0 1.2]);
        set(ax, 'FontSize', 8);
        if m < M,  set(ax, 'XTickLabel', {});  end
        if p == 1
            ylabel(ax, sprintf('Sp%d', m), 'FontSize', 7, ...
                'Rotation', 0, 'HorizontalAlignment', 'right');
        else
            set(ax, 'YTickLabel', {});
        end
    end
end

xlabel(tl, 'Time (s)', 'FontSize', 10);
if num_leaves > maxPaths
    fprintf('[plotAllPaths] %d leaves total, showing cheapest %d.\n', num_leaves, maxPaths);
end

end


% ── Local helpers ─────────────────────────────────────────────────────────

function pathNodes = tracePath_(NODES, leaf_idx)
    pathNodes = [];
    idx = leaf_idx;
    while idx > 0
        pathNodes = [idx, pathNodes];
        idx = NODES{idx}{7};
    end
end


function [t_w, route_usage, resc_occ, space_usage, gen_times, alpha_arr, xAlloc] = ...
         extractPathData_(NODES, pathNodes, cfg)

N = cfg.N;  M = cfg.M;  C = cfg.C;  Ni = cfg.Ni;

% Patch remaining times
for i = 1:length(NODES)
    node = NODES{i};
    for n = 1:N
        if node{2}(1,n) == 0
            node{3}(:,n) = C(:,n);
        end
    end
    NODES{i} = node;
end

num_pts   = length(pathNodes);
t_w       = zeros(1, num_pts);
route_usage = zeros(N, num_pts);
resc_occ    = zeros(N, num_pts);
space_usage = zeros(M, num_pts);
gen_times   = cell(N, 1);

gamma_last = NODES{pathNodes(end)}{11};
alpha_arr  = cell(1, N);
for n = 1:N
    for i = 1:Ni(n)
        alpha_arr{n}(i) = gamma_last{n}(i) - sum(C(:,n));
    end
end
xAlloc = NODES{pathNodes(end)}{15};

for idx = 1:num_pts
    cur = NODES{pathNodes(idx)};
    t_w(idx) = cur{5};
    if idx == num_pts,  continue;  end

    nxt = NODES{pathNodes(idx+1)};
    V_nxt_all = nxt{9};

    for n = 1:N
        d       = cur{2}(n);
        d_next  = nxt{2}(n);
        remain  = cur{3}(:,n);
        remain_next = nxt{3}(:,n);
        V_next  = nxt{9}(n,:);

        if abs(d) < 1e-5 && d_next < 100
            if any(V_next > 0)
                route_usage(n,idx) = 1;
                resc_occ(n,idx)    = find(V_next > 0, 1);
            else
                route_usage(n,idx) = 0.5;
            end
            gen_times{n} = [gen_times{n}, cur{5}];
        elseif abs(d) > 1e-5 && any(V_next > 0)
            route_usage(n,idx) = 1;
            resc_occ(n,idx)    = find(V_next > 0, 1);
        elseif abs(d) > 1e-5 && abs(d_next) < 1e-5
            route_usage(n,idx) = double(sum(remain) > 1e-5) * 0.5;
        elseif abs(d) > 1e-3 && abs(d_next) > 1e-3 && abs(d) < 100 && ...
               ((sum(remain) < sum(remain_next) && all(V_next==0)) || ...
                (sum(remain) == sum(remain_next) && abs(sum(remain)) > 1e-3))
            route_usage(n,idx) = 0.5;
        end
    end

    for m = 1:M
        if any(V_nxt_all(:,m) > 0)
            space_usage(m,idx) = 1;
        end
    end
end

end
