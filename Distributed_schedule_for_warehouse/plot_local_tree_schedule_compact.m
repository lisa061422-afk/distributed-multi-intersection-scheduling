function fig = plot_local_tree_schedule_compact(NODES, Path_min, Cmat, agent_i, valid_systems, pathInfo, pos_map, varargin)
%PLOT_LOCAL_TREE_SCHEDULE_COMPACT  Compact local decision-tree schedule plot for one intersection agent.
%
% Usage:
%   fig = plot_local_tree_schedule_compact(NODES, Path_min, Cmat, agent_i, valid_systems, pathInfo, pos_map);
%
% Inputs:
%   NODES         : cell array of nodes (as in INi_Admm_DecisionTree)
%   Path_min      : vector of node indices from root -> chosen leaf
%   Cmat          : S x N matrix, local durations (space sequence durations per system)
%   agent_i       : current agent index (only used for title/traceability)
%   valid_systems : vector of system indices participating in this local agent
%   pathInfo      : 1xN cell, each is struct with field .routeId (your pathInfo{n}.routeId)
%   pos_map       : 1xN cell, pos_map{n}(kn)=posi (your code uses kn=1)
%
% Name-Value options:
%   't1'          : xlim start (default: earliest alpha)
%   'tf'          : xlim end   (default: latest gamma)
%   'showReset'   : true/false (default: true)
%   'colors'      : cell array of RGB (default: your original colors)
%   'routeLabelMode' : 'routeId' or 'systemId' (default: 'routeId')
%   'gap'         : vertical gap between subplots (default: 0.010)
%   'marg_h'      : top/bottom margin (default: 0.045)
%   'marg_w'      : left/right margin (default: 0.080)
%   'figColor'    : figure background color (default: [1 1 1])
%   'axColor'     : axes background color (default: [1 1 1])
%   'lineColor'   : stairs line color (default: [0.53 0.81 0.92])
%
% Output:
%   fig : figure handle

% -------------------- options --------------------
p = inputParser;
p.addParameter('t1', [], @(x) isempty(x) || isscalar(x));
p.addParameter('tf', [], @(x) isempty(x) || isscalar(x));
p.addParameter('showReset', true, @(x) islogical(x) || ismember(x,[0 1]));
p.addParameter('colors', [], @(x) isempty(x) || iscell(x));
p.addParameter('routeLabelMode', 'routeId', @(s) ischar(s) || isstring(s));
p.addParameter('gap', 0.010, @(x) isscalar(x) && x>=0);
p.addParameter('marg_h', 0.045, @(x) isscalar(x) && x>=0);
p.addParameter('marg_w', 0.080, @(x) isscalar(x) && x>=0);
p.addParameter('figColor', [1 1 1], @(x) isnumeric(x) && numel(x)==3);
p.addParameter('axColor',  [1 1 1], @(x) isnumeric(x) && numel(x)==3);
p.addParameter('lineColor',[0.53 0.81 0.92], @(x) isnumeric(x) && numel(x)==3);
p.addParameter('title_pad', 0.055, @(x) isscalar(x) && x>=0);  % 给sgtitle留空间

p.parse(varargin{:});
opt = p.Results;

if isempty(opt.colors)
    opt.colors = {[0 1 0], [1 0.85 0], [1 0.5 0], [0 0.7 0.9], [1 0 0], [1 1 1]};
end

% -------------------- sizes --------------------
N = size(Cmat, 2);
Nv = numel(valid_systems);
num_points = numel(Path_min);

% infer M from V matrix of any transition node
M = 0;
if num_points >= 2
    Vtmp = NODES{Path_min(2)}{9};
    if ~isempty(Vtmp)
        M = size(Vtmp, 2);
    end
end

% -------------------- compute route labels --------------------
routeLabels = nan(1, N);  % only valid_systems meaningful
mode = lower(string(opt.routeLabelMode));

for n = valid_systems(:)'
    if mode == "systemid"
        routeLabels(n) = n;
        continue;
    end

    % kn=1 (your current code)
    if isempty(pos_map{n}) || numel(pos_map{n}) < 1 || isnan(pos_map{n}(1))
        routeLabels(n) = n;
        continue;
    end
    posi = pos_map{n}(1);
    pos_int = (posi + 1)/2;

    if isempty(pathInfo{n}) || ~isfield(pathInfo{n}, 'routeId')
        routeLabels(n) = n;
        continue;
    end

    rid = pathInfo{n}.routeId;
    if pos_int >= 1 && pos_int <= numel(rid)
        routeLabels(n) = rid(pos_int);
    else
        routeLabels(n) = n; % fallback
    end
end

% -------------------- pre-alloc traces --------------------
t_w             = zeros(1, num_points);
route_usage     = zeros(N, num_points);
resc_occupation = zeros(N, num_points);
space_usage     = zeros(M, num_points);
generation_times = cell(N, 1);

% -------------------- alpha from last node --------------------
gamma_lastnode = NODES{Path_min(end)}{11};   % gamma{n}(i)
alpha = cell(1, N);
for n = 1:N
    if isempty(gamma_lastnode) || numel(gamma_lastnode) < n || isempty(gamma_lastnode{n})
        alpha{n} = [];
        continue;
    end
    alpha{n} = gamma_lastnode{n} - sum(Cmat(:, n));
end

% x reset info (draw at leaf)
x_reset = NODES{Path_min(end)}{15};

% -------------------- scan along chosen path --------------------
for idx = 1:num_points
    cidx = Path_min(idx);
    cnode = NODES{cidx};
    t_w(idx) = cnode{5};

    if idx >= num_points
        break;
    end

    nidx = Path_min(idx+1);
    nnode = NODES{nidx};

    V_next_whole = nnode{9};  % N x M

    for n = valid_systems(:)'  % only evaluate valid systems
        d      = cnode{2}(n);
        d_next = nnode{2}(n);

        remain      = cnode{3}(:, n);
        remain_next = nnode{3}(:, n);

        V_next = nnode{9}(n, :);

        if abs(d) < 1e-5 && d_next < 100
            % new task generated
            if any(V_next) > 0
                route_usage(n,idx) = 1;
                resc_occupation(n,idx) = find(V_next > 0, 1, 'first');
            else
                route_usage(n,idx) = 0.5;
                resc_occupation(n,idx) = 0;
            end
            generation_times{n} = [generation_times{n}, cnode{5}];

        elseif abs(d) > 1e-5 && any(V_next) > 0
            route_usage(n,idx) = 1;
            resc_occupation(n,idx) = find(V_next > 0, 1, 'first');

        elseif abs(d) > 1e-5 && abs(d_next) < 1e-5
            if sum(remain) > 1e-5
                route_usage(n,idx) = 0.5;
            else
                route_usage(n,idx) = 0;
            end

        elseif abs(d) > 1e-3 && abs(d_next) > 1e-3 && abs(d) < 100 && ...
                ( (sum(remain) < sum(remain_next) && all(V_next == 0)) || ...
                  (sum(remain) == sum(remain_next) && abs(sum(remain)) > 1e-3) )
            route_usage(n,idx) = 0.5;

        else
            route_usage(n,idx) = 0;
        end
    end

    % space usage: if any system occupies space m at this transition, mark 1
    for m = 1:M
        if isempty(V_next_whole)
            space_usage(m,idx) = 0;
        else
            col = V_next_whole(:, m);
            space_usage(m,idx) = ~all(col == 0);
        end
    end
end

% --------- default xlim based on earliest enter and latest leave ----------
if isempty(opt.t1) || isempty(opt.tf)
    enterTimes = [];
    leaveTimes = [];

    for n = valid_systems(:)'
        if ~isempty(alpha{n})
            enterTimes = [enterTimes, alpha{n}(:)']; %#ok<AGROW>
        end
        if ~isempty(gamma_lastnode) && numel(gamma_lastnode) >= n && ~isempty(gamma_lastnode{n})
            leaveTimes = [leaveTimes, gamma_lastnode{n}(:)']; %#ok<AGROW>
        end
    end

    enterTimes = enterTimes(~isnan(enterTimes));
    leaveTimes = leaveTimes(~isnan(leaveTimes));

    if ~isempty(enterTimes) && ~isempty(leaveTimes)
        tMin = min(enterTimes);
        tMax = max(leaveTimes);
    else
        tMin = min(t_w);
        tMax = max(t_w);
    end

    pad = 0.05*(tMax - tMin + eps);   % 5% padding
    if isempty(opt.t1), opt.t1 = tMin - pad; end
    if isempty(opt.tf), opt.tf = tMax + pad; end
end

% -------------------- plotting --------------------
% fig = figure('Color', opt.figColor); 
fig = figure('Color', opt.figColor,...
    'Units','normalized', 'Position',[0.08 0.08 0.55 0.85]);  % 更高更大
rows = Nv + M;

% plot per system (only valid systems)
for ii = 1:Nv
    n = valid_systems(ii);

    subplot(rows, 1, ii);
    ax = gca; hold(ax,'on');
    ax.FontSize = 16;
    ax.Color    = opt.axColor;
    box(ax,'on'); grid(ax,'on');

    for idx = 1:num_points-1
        x_start = t_w(idx);
        x_end   = t_w(idx + 1);
        y_value = route_usage(n, idx);

        if y_value > 0.7
            color_index = resc_occupation(n,idx);
            if color_index >= 1 && color_index <= numel(opt.colors)
                color = opt.colors{color_index};
                fill([x_start, x_end, x_end, x_start], [0, 0, y_value, y_value], ...
                    color, 'EdgeColor', 'none');
            end
        end
    end

    stairs(t_w, route_usage(n, :), 'LineWidth', 2, 'Color', opt.lineColor);

    if ~isnan(routeLabels(n))
        ylabel(sprintf('V%d-R%d', n, routeLabels(n)), 'FontSize', 16, 'Rotation', 0);
    else
        ylabel(sprintf('V%d', n), 'FontSize', 16, 'Rotation', 0);
    end

    % generation times
    if ~isempty(generation_times{n})
        for gen_time = generation_times{n}
            xline(gen_time, 'b-.', 'LineWidth', 1.8);
        end
    end

    xlim([opt.t1, opt.tf]);
    ylim([0 1.1]);
end

% plot per space
for m = 1:M
    subplot(rows, 1, Nv + m);
    ax = gca; hold(ax,'on');
    ax.FontSize = 16;
    ax.Color    = opt.axColor;
    box(ax,'on'); grid(ax,'on');

    for idx = 1:num_points-1
        x_start = t_w(idx);
        x_end   = t_w(idx + 1);
        y_value = space_usage(m, idx);

        if y_value > 0
            color = opt.colors{min(m, numel(opt.colors))};
            fill([x_start, x_end, x_end, x_start], [0, 0, y_value, y_value], ...
                color, 'EdgeColor', 'none');
        end
    end

    stairs(t_w, space_usage(m, :), 'LineWidth', 2, 'Color', opt.lineColor);
    ylabel(sprintf('S%d', m), 'FontSize', 16, 'Rotation', 0);

    xlim([opt.t1, opt.tf]);
    ylim([0 1.2]);
end

xlabel('Time (seconds)', 'FontSize', 16);
sgtitle(sprintf('Local Decision-Tree Schedule (Agent %d)', agent_i), 'FontSize', 16);

% -------------------- draw reset rectangles --------------------
if opt.showReset
    draw_reset_rectangles(valid_systems, Nv, M, opt.colors, x_reset);
end

% --------- tighten subplots (do this at very end) ----------
tight_subplots(fig, rows, 1, opt.gap, opt.marg_h, opt.marg_w, opt.title_pad);

end

% =================== helper functions ===================

function draw_reset_rectangles(valid_systems, Nv, M, colors, x_reset)
% Draw only the "last reset rectangle" for each unique (s,m) in each task (same as your original logic).
% Subplot indices: systems are 1..Nv, spaces are Nv+1 .. Nv+M

for ii = 1:numel(valid_systems)
    n = valid_systems(ii);

    if isempty(x_reset) || numel(x_reset) < n || isempty(x_reset{n})
        continue;
    end

    for k = 1:numel(x_reset{n})
        x_info = x_reset{n}{k};
        if isempty(x_info), continue; end

        seen = containers.Map();

        for row = size(x_info,1):-1:1
            m = x_info{row, 4};
            key = sprintf('%d-%d', x_info{row,3}, m);
            if ~isKey(seen, key)
                seen(key) = row;
            end
        end

        keys_seen = keys(seen);
        for jj = 1:numel(keys_seen)
            row = seen(keys_seen{jj});
            t_start = x_info{row, 1};
            t_end   = x_info{row, 2};
            m       = x_info{row, 4};

            if t_start == t_end, continue; end
            color = colors{min(m, numel(colors))};

            % system subplot (ii)
            subplot(Nv + M, 1, ii); hold on;
            fill([t_start, t_end, t_end, t_start], [0, 0, 1, 1], color, ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.65);

            % space subplot (Nv + m)
            if m >= 1 && m <= M
                subplot(Nv + M, 1, Nv + m); hold on;
                fill([t_start, t_end, t_end, t_start], [0, 0, 1, 1], color, ...
                    'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.65);
            end
        end
    end
end
end

function tight_subplots(fig, nRow, nCol, gap, marg_h, marg_w, title_pad)
% Tighten subplot spacing (works with subplot-generated axes).
% title_pad: extra normalized height reserved at top for sgtitle

if nargin < 7 || isempty(title_pad)
    title_pad = 0.055;
end

axs = findall(fig,'Type','axes');

% sgtitle 可能会创建额外对象，过滤掉 Tag=suptitle/sgtitle 的 axes
tags = get(axs,'Tag');
if ischar(tags), tags = {tags}; end
keep = true(size(axs));
for i = 1:numel(axs)
    if ischar(tags{i}) && (strcmp(tags{i},'suptitle') || strcmp(tags{i},'sgtitle'))
        keep(i) = false;
    end
end
axs = axs(keep);

axs = flipud(axs); % keep creation order

usableH = 1 - (marg_h + title_pad) - marg_h;     % 顶部多减 title_pad
axH = (usableH - (nRow-1)*gap) / nRow;
axW = (1 - 2*marg_w - (nCol-1)*gap) / nCol;

k = 0;
for r = 1:nRow
    for c = 1:nCol
        k = k + 1;
        if k > numel(axs), return; end
        left   = marg_w + (c-1)*(axW+gap);

        topEdge = 1 - (marg_h + title_pad);
        bottom  = topEdge - r*axH - (r-1)*gap;

        set(axs(k),'Position',[left bottom axW axH]);
    end
end
end
