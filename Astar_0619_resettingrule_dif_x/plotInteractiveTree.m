function plotInteractiveTree(NODES, LEAF, cfg)
% plotInteractiveTree  Click a leaf on the tree → resource allocation updates.
%
%   Left  : decision tree  (green=optimal, orange=other leaves, blue=rest)
%   Right : N+M stacked subplots, same style as ResouceAllocation_Traffic_Plot
%   Click any leaf node to view its schedule.

N = cfg.N;  M = cfg.M;
nRows = N + M;

% ── Sort leaves by g-cost ─────────────────────────────────────────────
gcosts = arrayfun(@(l) NODES{l}{10}, LEAF);
[gcosts_s, sidx] = sort(gcosts, 'ascend');
LEAF_s = LEAF(sidx);

% ── Build digraph ─────────────────────────────────────────────────────
num_nodes = size(NODES, 1);
edges = [];
for i = 1:num_nodes
    p = NODES{i}{7};
    if p ~= 0,  edges = [edges; p, i];  end
end
G = digraph(edges(:,1), edges(:,2));

% ── Figure ────────────────────────────────────────────────────────────
if cfg.useWeakRule
    rule_str = 'WeakRule: ON';   rule_col = [0.05 0.55 0.05];
else
    rule_str = 'WeakRule: OFF';  rule_col = [0.80 0.10 0.10];
end
fig = figure('Name', sprintf('Path Explorer  —  %s', rule_str), ...
             'Position', [40 40 1550 760]);

% ── Tree axes (left 38%) ──────────────────────────────────────────────
ax_t = axes(fig, 'Units','normalized', 'Position', [0.02 0.05 0.36 0.90]);
h = plot(ax_t, G, 'Layout','layered', 'NodeLabel', {});
h.XData = h.XData * 3;
h.YData = h.YData * 1.8;
hold(ax_t, 'on');

nc = repmat([0.55 0.78 0.95], num_nodes, 1);
for i = 2:length(LEAF_s),  nc(LEAF_s(i),:) = [0.95 0.60 0.25];  end
nc(LEAF_s(1),:) = [0.25 0.82 0.35];
h.NodeColor = nc;  h.MarkerSize = 6;

max_labels = 30;
for i = 1:min(length(LEAF_s), max_labels)
    l = LEAF_s(i);
    lbl = sprintf('#%d  g=%.2f', i, gcosts_s(i));
    text(ax_t, h.XData(l)+0.12, h.YData(l)+0.22, lbl, 'FontSize', 7);
end
title(ax_t, sprintf('Click a leaf  (%d total)   [%s]', length(LEAF_s), rule_str), ...
    'FontSize', 9, 'Color', rule_col, 'FontWeight', 'bold');
axis(ax_t,'tight');  box(ax_t,'on');

% ── Resource-allocation axes (right 56%, N+M rows) ───────────────────
ax_x = 0.42;  ax_w = 0.56;
ax_h = 0.90 / nRows;
ra_axes = gobjects(nRows, 1);
for r = 1:nRows
    bot = 0.05 + (nRows - r) * ax_h;
    ra_axes(r) = axes(fig, 'Units','normalized', ...
        'Position', [ax_x, bot, ax_w, ax_h * 0.90]);
    set(ra_axes(r), 'FontSize', 9);
    if r < nRows,  set(ra_axes(r), 'XTickLabel', {});  end
end
xlabel(ra_axes(nRows), 'Time (s)', 'FontSize', 11);

% Placeholder titles
for r = 1:N
    ylabel(ra_axes(r), sprintf('N%d', r), ...
        'FontSize', 8, 'Rotation', 0, 'HorizontalAlignment', 'right');
end
for m = 1:M
    ylabel(ra_axes(N+m), sprintf('M%d', m), ...
        'FontSize', 8, 'Rotation', 0, 'HorizontalAlignment', 'right');
end
title(ra_axes(1), 'Select a leaf node on the tree', 'FontSize', 10);

% ── Store state ───────────────────────────────────────────────────────
fig.UserData = struct('NODES',{NODES},'LEAF_s',LEAF_s,'gcosts_s',gcosts_s, ...
    'cfg',cfg,'h',h,'ax_t',ax_t,'ra_axes',ra_axes,'num_nodes',num_nodes);

fig.WindowButtonDownFcn = @figClickCb;
fig.KeyPressFcn         = @keyPressCb;    % ← → arrow keys to cycle leaves

% Show optimal by default
showPath(fig, LEAF_s(1), 1);
figure(fig);   % bring to front and capture keyboard focus
end


% ═══════════════════════════════════════════════════════════════════════
function keyPressCb(fig, evt)
    ud = fig.UserData;
    if ~isfield(ud, 'currentRank'),  ud.currentRank = 1;  end
    n_leaves = length(ud.LEAF_s);
    switch evt.Key
        case 'rightarrow'
            newRank = min(ud.currentRank + 1, n_leaves);
        case 'leftarrow'
            newRank = max(ud.currentRank - 1, 1);
        otherwise
            return;
    end
    ud.currentRank  = newRank;
    fig.UserData    = ud;
    showPath(fig, ud.LEAF_s(newRank), newRank);
end


% ═══════════════════════════════════════════════════════════════════════
function figClickCb(fig, ~)
    ud = fig.UserData;

    % Verify click lands inside the tree axes (pixel bounds)
    fp = fig.Position;          % [x y w h] pixels on screen
    cp = fig.CurrentPoint;      % [x y] pixels within figure
    ap = ud.ax_t.Position;      % normalized [x y w h]
    if cp(1) < ap(1)*fp(3) || cp(1) > (ap(1)+ap(3))*fp(3) || ...
       cp(2) < ap(2)*fp(4) || cp(2) > (ap(2)+ap(4))*fp(4)
        return;
    end

    % Find nearest LEAF (not nearest arbitrary node) — no need for exact hit
    dp = ud.ax_t.CurrentPoint;
    cx = dp(1,1);   cy = dp(1,2);
    h  = ud.h;
    lx = h.XData(ud.LEAF_s);
    ly = h.YData(ud.LEAF_s);
    [~, leafRank] = min((lx - cx).^2 + (ly - cy).^2);
    showPath(fig, ud.LEAF_s(leafRank), leafRank);
end


% ═══════════════════════════════════════════════════════════════════════
function showPath(fig, leaf_idx, rank)
    ud  = fig.UserData;
    ud.currentRank = rank;
    fig.UserData   = ud;
    cfg = ud.cfg;
    N = cfg.N;  M = cfg.M;
    ra_axes = ud.ra_axes;

    % Trace path and extract data
    pathNodes = tracePath_(ud.NODES, leaf_idx);
    [t_w, route_usage, resc_occ, space_blocks, gen_times, alpha_arr, xAlloc] = ...
        extractPathData_(ud.NODES, pathNodes, cfg);

    % ── Highlight selected path on tree ──────────────────────────────────
    h          = ud.h;
    num_nodes  = ud.num_nodes;
    nc = repmat([0.55 0.78 0.95], num_nodes, 1);
    for i = 2:length(ud.LEAF_s),  nc(ud.LEAF_s(i),:) = [0.95 0.60 0.25];  end
    nc(ud.LEAF_s(1),:) = [0.25 0.82 0.35];
    h.NodeColor = nc;
    h.EdgeColor = [0.70 0.70 0.70];
    h.LineWidth = 0.5;
    if length(pathNodes) > 1
        highlight(h, pathNodes(1:end-1), pathNodes(2:end), ...
            'EdgeColor', [0.85 0.12 0.12], 'LineWidth', 2.5);
    end
    highlight(h, pathNodes, 'NodeColor', [0.85 0.12 0.12]);

    % Time window: always start from earliest alpha_tilde so approach is visible
    at_vals = cellfun(@(v) v(1), cfg.alpha_tilde);
    tMin_at = min(at_vals);
    if ~isempty(space_blocks)
        blk_starts = cellfun(@(b) b{1}, space_blocks);
        blk_ends   = cellfun(@(b) b{2}, space_blocks);
        tMin = min(min(blk_starts), tMin_at);
        tMax = max(blk_ends);
    else
        alpha_vals = cell2mat(cellfun(@(a) a(~isnan(a)), alpha_arr, 'UniformOutput', false));
        tw_finite  = t_w(isfinite(t_w) & t_w < 1e6);
        all_t = [alpha_vals, tw_finite, tMin_at];
        all_t = all_t(isfinite(all_t));
        if isempty(all_t),  all_t = [0 20];  end
        tMin = min(all_t);  tMax = max(all_t);
    end
    pad = 0.05 * max(tMax - tMin, 1);
    t1 = tMin - pad;
    tf = tMax + pad;

    np = length(t_w);
    colors = {[0 1 0],[1 0.85 0],[1 0.5 0],[0 0.7 0.9],[1 0 0],[1 1 1]};

    % ── System subplots ───────────────────────────────────────────────
    for n = 1:N
        ax = ra_axes(n);
        cla(ax);  hold(ax,'on');

        for idx = 1:np-1
            xs = t_w(idx);  xe = t_w(idx+1);
            yv = route_usage(n, idx);
            if yv > 0.7
                ci = resc_occ(n, idx);
                fill(ax, [xs xe xe xs], [0 0 yv yv], colors{ci}, 'EdgeColor','none');
            end
        end
        stairs(ax, t_w, route_usage(n,:), 'LineWidth', 2, 'Color', [0.53 0.81 0.92]);
        for gt = gen_times{n}
            xline(ax, gt, 'b-.', 'LineWidth', 1.5);
        end
        for at = alpha_arr{n}
            xline(ax, at, '-.', 'LineWidth', 1.5, 'Color', [1 0 0]);
        end
        ylabel(ax, sprintf('N%d', n), ...
            'FontSize', 8, 'Rotation', 0, 'HorizontalAlignment', 'right');
        xlim(ax,[t1 tf]);  ylim(ax,[0 1.1]);  grid(ax,'on');
    end

    % ── Space subplots (backtracked from gamma/alpha) ─────────────────
    for m = 1:M
        ax = ra_axes(N + m);
        cla(ax);  hold(ax,'on');
        col = colors{m};

        for bi = 1:numel(space_blocks)
            blk = space_blocks{bi};
            if blk{3} ~= m,  continue;  end
            ts = blk{1};  te = blk{2};  n_sys = blk{4};
            fill(ax, [ts te te ts], [0 0 1 1], [1 1 1], 'EdgeColor', col, 'LineWidth', 2);
            text(ax, (ts+te)/2, 0.55, sprintf('N%d', n_sys), ...
                'FontSize', 9, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'Color', col);
        end

        ylabel(ax, sprintf('M%d', m), ...
            'FontSize', 8, 'Rotation', 0, 'HorizontalAlignment', 'right');
        xlim(ax,[t1 tf]);  ylim(ax,[0 1.2]);  grid(ax,'on');
    end

    % ── Resetting-rule rectangles (black border, semi-transparent) ────
    for n = 1:N
        if isempty(xAlloc{n}),  continue;  end
        for k = 1:length(xAlloc{n})
            xi = xAlloc{n}{k};
            if isempty(xi),  continue;  end
            seen = containers.Map();
            for row = size(xi,1):-1:1
                key = sprintf('%d-%d', xi{row,3}, xi{row,4});
                if ~isKey(seen,key),  seen(key) = row;  end
            end
            for ky = keys(seen)
                row = seen(ky{1});
                ts  = xi{row,1};  te  = xi{row,2};
                m   = xi{row,4};
                if ts == te,  continue;  end
                col = colors{m};
                % system subplot only (reset rectangles not drawn in space rows)
                fill(ra_axes(n), [ts te te ts], [0 0 1 1], col, ...
                    'EdgeColor','k', 'LineWidth', 1.2, 'FaceAlpha', 0.7);
            end
        end
    end

    % ── Title on top subplot ──────────────────────────────────────────
    gcost = ud.gcosts_s(rank);
    if rank == 1
        ttl = sprintf('Path %d  |  g = %.4f  ★ OPTIMAL', rank, gcost);
        title(ra_axes(1), ttl, 'FontSize', 10, 'FontWeight','bold', 'Color',[0.1 0.5 0.1]);
    else
        ttl = sprintf('Path %d  |  g = %.4f    (optimal: %.4f)', rank, gcost, ud.gcosts_s(1));
        title(ra_axes(1), ttl, 'FontSize', 10, 'Color',[0.15 0.15 0.65]);
    end

    % Highlight selected leaf on tree
    delete(findobj(ud.ax_t, 'Tag', 'sel_marker'));
    plot(ud.ax_t, ud.h.XData(leaf_idx), ud.h.YData(leaf_idx), 'yo', ...
        'MarkerSize', 14, 'LineWidth', 2.5, 'Tag', 'sel_marker');
end


% ═══════════════════════════════════════════════════════════════════════
function pathNodes = tracePath_(NODES, leaf_idx)
    pathNodes = [];
    idx = leaf_idx;
    while idx > 0
        pathNodes = [idx, pathNodes];
        idx = NODES{idx}{7};
    end
end


% ═══════════════════════════════════════════════════════════════════════
function [t_w, route_usage, resc_occ, space_blocks, gen_times, alpha_arr, xAlloc] = ...
         extractPathData_(NODES, pathNodes, cfg)

N = cfg.N;  M = cfg.M;  C = cfg.C;  Ni = cfg.Ni;  Map = cfg.Map;  S = cfg.S;

for i = 1:length(NODES)
    node = NODES{i};
    for n = 1:N
        if node{2}(1,n) == 0,  node{3}(:,n) = C(:,n);  end
    end
    NODES{i} = node;
end

num_pts     = length(pathNodes);
t_w         = zeros(1, num_pts);
route_usage = zeros(N, num_pts);
resc_occ    = zeros(N, num_pts);
gen_times   = cell(N, 1);

gamma_last = NODES{pathNodes(end)}{11};
alpha_arr  = cell(1, N);
for n = 1:N
    alpha_arr{n} = nan(1, Ni(n));
    for i = 1:Ni(n)
        if ~isempty(gamma_last{n}) && numel(gamma_last{n}) >= i && gamma_last{n}(i) > 0
            alpha_arr{n}(i) = gamma_last{n}(i) - sum(C(:,n));
        end
    end
end
xAlloc = NODES{pathNodes(end)}{15};

% ── space blocks backtracked from gamma/alpha ─────────────────────────
space_blocks = {};
for n = 1:N
    for i = 1:Ni(n)
        if isnan(alpha_arr{n}(i)),  continue;  end
        alpha_n = alpha_arr{n}(i);
        t_acc   = 0;
        for s = 1:S
            m = Map(s, n);
            if m > 0 && C(s,n) > 1e-9
                space_blocks{end+1} = {alpha_n + t_acc, alpha_n + t_acc + C(s,n), m, n}; %#ok<AGROW>
            end
            t_acc = t_acc + C(s,n);
        end
    end
end

for idx = 1:num_pts
    cur      = NODES{pathNodes(idx)};
    t_w(idx) = cur{5};
    if idx == num_pts,  continue;  end
    nxt = NODES{pathNodes(idx+1)};

    for n = 1:N
        d           = cur{2}(n);    d_next      = nxt{2}(n);
        remain      = cur{3}(:,n);  remain_next = nxt{3}(:,n);
        V_next      = nxt{9}(n,:);

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
end

end
