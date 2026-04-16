function fig = plot_local_tree_schedule_compact_centralized( ...
    NODES, Path_min, DATA, const, agent_i, valid_systems, varargin)
%PLOT_LOCAL_TREE_SCHEDULE_COMPACT_CENTRALIZED
% Centralized adaptation of your original plot_local_tree_schedule_compact
% with minimal necessary modifications only.
%
% Changes relative to original:
%   1) Use local V columns for the current intersection only
%   2) Use DATA.alpha_local{agent_i}(n) as generation times
%   3) Keep reset rectangles as semi-transparent colored fill + black border
%
% Everything else follows your original design/logic as closely as possible.

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
p.addParameter('title_pad', 0.055, @(x) isscalar(x) && x>=0);

p.parse(varargin{:});
opt = p.Results;

if isempty(opt.colors)
    opt.colors = {[0 1 0], [1 0.85 0], [1 0.5 0], [0 0.7 0.9], [1 0 0], [1 1 1]};
end

% -------------------- sizes --------------------
N = const.N;
Nv = numel(valid_systems);
num_points = numel(Path_min);

% current intersection local space columns
M = const.spacePerInt;
local_cols = (agent_i-1)*M + (1:M);

% -------------------- compute route labels --------------------
routeLabels = nan(1, N);
mode = lower(string(opt.routeLabelMode));

for n = valid_systems(:)'
    if mode == "systemid"
        routeLabels(n) = n;
        continue;
    end

    pos_int = find(const.Veh(n).intSeq == agent_i, 1, 'first');
    if isempty(pos_int)
        routeLabels(n) = n;
        continue;
    end

    if isfield(const.Veh(n), 'routeId') && numel(const.Veh(n).routeId) >= pos_int
        routeLabels(n) = const.Veh(n).routeId(pos_int);
    else
        routeLabels(n) = n;
    end
end

% -------------------- pre-alloc traces --------------------
t_w              = zeros(1, num_points);
route_usage      = zeros(N, num_points);
resc_occupation  = zeros(N, num_points);
space_usage      = zeros(M, num_points);
generation_times = cell(N, 1);

% -------------------- alpha from DATA (minimal centralized adaptation) --------------------
gamma_lastnode = NODES{Path_min(end)}{11};   % keep original variable name
alpha_local = DATA.alpha_local{agent_i};
for n = valid_systems(:)'
    if numel(alpha_local) >= n && isfinite(alpha_local(n))
        generation_times{n} = alpha_local(n);
    else
        generation_times{n} = [];
    end
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

    nidx = Path_min(idx+1);%next indx
    nnode = NODES{nidx};

    V_next_whole = nnode{9};  % N x Mtot
    if isempty(V_next_whole)
        V_next_local_whole = zeros(N, M);
    else
        V_next_local_whole = V_next_whole(:, local_cols);
    end

    for n = valid_systems(:)'  % only evaluate valid systems
        d      = cnode{2}(n);
        d_next = nnode{2}(n);

        remain      = cnode{3}(:, n);
        remain_next = nnode{3}(:, n);

        % ===== only necessary change: use LOCAL V for this intersection =====
        if size(V_next_local_whole,1) >= n
            V_next = V_next_local_whole(n, :);
        else
            V_next = zeros(1, M);
        end
        
        task_idx = nnode{6}(n);   % 当前区间对应的 task index
        is_this_agent_task = false;

        if task_idx >= 1 && task_idx <= const.NI(n)
            is_this_agent_task = (const.Veh(n).intSeq(task_idx) == agent_i);
        end
        % ===== keep your original logic exactly =====
        if is_this_agent_task && abs(d) < 1e-5 && d_next < 1e7
            % new task generated
            if any(V_next > 0)
                route_usage(n,idx) = 1;
                resc_occupation(n,idx) = find(V_next > 0, 1, 'first');
            else
                route_usage(n,idx) = 0.5;
                resc_occupation(n,idx) = 0;
            end

        elseif abs(d) > 1e-5 && any(V_next > 0)
            route_usage(n,idx) = 1;
            resc_occupation(n,idx) = find(V_next > 0, 1, 'first');

        elseif abs(d) > 1e-5 && abs(d_next) < 1e-5
            if sum(remain) > 1e-5
                route_usage(n,idx) = 0.5;
            else
                route_usage(n,idx) = 0;
            end

        elseif abs(d) > 1e-7 && abs(d_next) > 1e-7 && abs(d) < 100 && ...
                ( (sum(remain) < sum(remain_next) && all(V_next == 0)) || ...
                  (sum(remain) == sum(remain_next) && abs(sum(remain)) > 1e-3) )
            route_usage(n,idx) = 0.5;

        else
            route_usage(n,idx) = 0;
        end
    end

    % space usage: only current intersection local spaces
    for m = 1:M
        if isempty(V_next_local_whole)
            space_usage(m,idx) = 0;
        else
            col = V_next_local_whole(:, m);
            space_usage(m,idx) = ~all(col == 0);
        end
    end
end

% --------- default xlim based on earliest enter and latest leave ----------
if isempty(opt.t1) || isempty(opt.tf)
    enterTimes = [];
    leaveTimes = [];

    gamma_local = DATA.gamma_local{agent_i};

    for n = valid_systems(:)'
        if numel(alpha_local) >= n && isfinite(alpha_local(n))
            enterTimes = [enterTimes, alpha_local(n)]; %#ok<AGROW>
        end
        if numel(gamma_local) >= n && isfinite(gamma_local(n))
            leaveTimes = [leaveTimes, gamma_local(n)]; %#ok<AGROW>
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
fig = figure('Color', opt.figColor,...
    'Units','normalized', 'Position',[0.08 0.08 0.55 0.85]);
rows = Nv + M;

% plot per system (only valid systems)
for ii = 1:Nv
    n = valid_systems(ii);

    subplot(rows, 1, ii);
    ax = gca; cla(ax); hold(ax,'on');
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

    k_here = find(const.Veh(n).intSeq == agent_i, 1, 'first');
    if ~isempty(k_here)
        ylabel(sprintf('V%d-K%d', n, k_here), 'FontSize', 16, 'Rotation', 0);
    elseif ~isnan(routeLabels(n))
        ylabel(sprintf('V%d-R%d', n, routeLabels(n)), 'FontSize', 16, 'Rotation', 0);
    else
        ylabel(sprintf('V%d', n), 'FontSize', 16, 'Rotation', 0);
    end

    % generation times
    if ~isempty(generation_times{n})
        for gen_time = generation_times{n}
            xline(gen_time, 'b--', 'LineWidth', 1.8);
        end
    end

    xlim([opt.t1, opt.tf]);
    ylim([0 1.1]);
end

% plot per space
for m = 1:M
    subplot(rows, 1, Nv + m);
    ax = gca; cla(ax); hold(ax,'on');
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
    draw_reset_rectangles_local(valid_systems, Nv, M, opt.colors, x_reset, local_cols);
end

% --------- tighten subplots (do this at very end) ----------
tight_subplots(fig, rows, 1, opt.gap, opt.marg_h, opt.marg_w, opt.title_pad);

end

% =================== helper functions ===================

function draw_reset_rectangles_local(valid_systems, Nv, M, colors, x_reset, local_cols)
% Same logic as your original helper, except:
%   - only keep rectangles that belong to current intersection local cols
%   - fill color stays semi-transparent
%   - black border preserved

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
            global_m = x_info{row, 4};
            local_m = find(local_cols == global_m, 1, 'first');
            if isempty(local_m)
                continue;
            end
            key = sprintf('%d-%d', x_info{row,3}, local_m);
            if ~isKey(seen, key)
                seen(key) = row;
            end
        end

        keys_seen = keys(seen);
        for jj = 1:numel(keys_seen)
            row = seen(keys_seen{jj});
            t_start = x_info{row, 1};
            t_end   = x_info{row, 2};
            global_m = x_info{row, 4};
            m = find(local_cols == global_m, 1, 'first');

            if isempty(m) || t_start == t_end
                continue;
            end
            color = colors{min(m, numel(colors))};

            % system subplot (ii)
            subplot(Nv + M, 1, ii); hold on;
            fill([t_start, t_end, t_end, t_start], [0, 0, 1, 1], color, ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.65);

            % space subplot (Nv + m)
            subplot(Nv + M, 1, Nv + m); hold on;
            fill([t_start, t_end, t_end, t_start], [0, 0, 1, 1], color, ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.65);
        end
    end
end
end

function tight_subplots(fig, nRow, nCol, gap, marg_h, marg_w, title_pad)
if nargin < 7 || isempty(title_pad)
    title_pad = 0.055;
end

axs = findall(fig,'Type','axes');

tags = get(axs,'Tag');
if ischar(tags), tags = {tags}; end
keep = true(size(axs));
for i = 1:numel(axs)
    if ischar(tags{i}) && (strcmp(tags{i},'suptitle') || strcmp(tags{i},'sgtitle'))
        keep(i) = false;
    end
end
axs = axs(keep);

axs = flipud(axs);

usableH = 1 - (marg_h + title_pad) - marg_h;
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