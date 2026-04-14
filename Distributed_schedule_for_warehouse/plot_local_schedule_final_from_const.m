function fig = plot_local_schedule_final_from_const( ...
    NODES, Path_min, agent_i, valid_systems, const, varargin)
% plot_local_schedule_final_from_const
% Draw final-only local schedule at one intersection using const.IntSpaceDB
% and const.pathInfo/pathInfo_agent_chain.
%
% Upper rows  : vehicle-wise final occupation result
% Lower rows  : space-wise final occupation result
%
% Vehicle rows:
%   - filled colored rectangles
%   - stairs outline
%
% Space rows:
%   - outlined rectangles (same color as the space)
%   - thicker edges
%   - vehicle label inside each block
%   - stairs outline
%
% INPUTS
%   NODES, Path_min : local tree cache result
%   agent_i         : current intersection index
%   valid_systems   : participating vehicle indices at this intersection
%   const           : must contain
%                       .IntSpaceDB
%                       .pathInfo
%                       .pathInfo_agent_chain
%
% OPTIONAL
%   'figSize'       : [w h], default [500 560]
%   'gap'           : default 0.004
%   'figColor'      : default [0.97 0.97 0.97]
%   'axColor'       : default [0.98 0.98 0.98]
%   'title_pad'     : default 0.03
%   'marg_h'        : default 0.05
%   'marg_w'        : default 0.08
%   'lineColor'     : stairs color
%   'spaceColors'   : numSpaces x 3 colormap

% -------------------- parse inputs --------------------
p = inputParser;
addParameter(p, 'figSize', [550 500]);
addParameter(p, 'gap', 0.004);
addParameter(p, 'figColor', [0.97 0.97 0.97]);
addParameter(p, 'axColor',  [0.98 0.98 0.98]);
addParameter(p, 'title_pad', 0.03);
addParameter(p, 'marg_h', 0.05);
addParameter(p, 'marg_w', 0.08);
addParameter(p, 'lineColor', [0.35 0.75 0.95]);
addParameter(p, 'spaceColors', [ ...
    0.16 0.95 0.16;   % S1 green
    0.98 0.84 0.00;   % S2 yellow
    1.00 0.55 0.00;   % S3 orange
    0.12 0.66 0.84;   % S4 cyan-blue
    0.95 0.10 0.10]); % S5 red
parse(p, varargin{:});
opt = p.Results;

IntSpaceDB = const.IntSpaceDB;
pathInfo = const.pathInfo;
pathInfo_agent_chain = const.pathInfo_agent_chain;

numSpaces = IntSpaceDB{agent_i}.numSpaces;
kn = 1;   % current setting: one vehicle/task per system

% -------------------- final gamma extraction --------------------
gamma_last = extract_final_gamma(NODES, Path_min);

% -------------------- build final schedule items --------------------
items = struct('veh', {}, 'k', {}, 'routeId', {}, ...
               'spaces', {}, 'dur', {}, ...
               't0', {}, 'tf', {}, ...
               'segStart', {}, 'segEnd', {});

for idx = 1:numel(valid_systems)
    n = valid_systems(idx);

    chain = pathInfo_agent_chain{n}{kn};
    posi = find(chain == agent_i, 1, 'first');
    if isempty(posi)
        continue;
    end

    % current agent is the k-th visited intersection of this vehicle
    pos_int = (posi + 1) / 2;

    % current route index at this intersection
    routeId = pathInfo{n}.routeId(pos_int);

    % local space sequence and durations
    space_seq = IntSpaceDB{agent_i}.routeSpace{routeId};
    dur_seq   = IntSpaceDB{agent_i}.routeDur{routeId};

    if isempty(space_seq) || isempty(dur_seq)
        continue;
    end

    % final completion time
    tf = fetch_vehicle_gamma(gamma_last, n, idx);
    if isempty(tf) || ~isfinite(tf)
        continue;
    end

    t0 = tf - sum(dur_seq);

    segStart = zeros(1, numel(dur_seq));
    segEnd   = zeros(1, numel(dur_seq));
    tcur = t0;
    for q = 1:numel(dur_seq)
        segStart(q) = tcur;
        segEnd(q)   = tcur + dur_seq(q);
        tcur = segEnd(q);
    end

    items(end+1) = struct( ...
        'veh', n, ...
        'k', pos_int, ...
        'routeId', routeId, ...
        'spaces', space_seq, ...
        'dur', dur_seq, ...
        't0', t0, ...
        'tf', tf, ...
        'segStart', segStart, ...
        'segEnd', segEnd); %#ok<AGROW>
end

if isempty(items)
    warning('No drawable final schedule found for agent %d.', agent_i);
    fig = [];
    return;
end

% sort by vehicle index
[~, ord] = sort([items.veh]);
items = items(ord);

% -------------------- figure layout --------------------
Nv  = numel(items);
Ns  = numSpaces;
Nax = Nv + Ns;

fig = figure('Color', opt.figColor, ...
    'Name', sprintf('Final Local Schedule (Intersection %d)', agent_i), ...
    'Position', [120 80 opt.figSize(1) opt.figSize(2)]);

axH = (1 - 2*opt.marg_h - (Nax-1)*opt.gap - opt.title_pad) / Nax;
axW = 1 - 2*opt.marg_w;

axes_list = gobjects(Nax,1);

xmin = min([items.t0]) - 0.4;
xmax = max([items.tf]) + 0.4;

% -------------------- vehicle rows --------------------
for r = 1:Nv
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', fig, 'Position', [opt.marg_w bottom axW axH]);
    axes_list(r) = ax;

    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, ...
        'FontSize', 9, ...
        'Box', 'on', ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.16, ...
        'Layer', 'bottom');

    y0 = 0;
    h  = 1.0;

    it = items(r);

    % --- draw baseline/outline first ---
    t_edges = [xmin, sort([it.segStart, it.segEnd]), xmax];
    t_edges = unique(t_edges);
    occ = zeros(size(t_edges));
    for kk = 1:numel(t_edges)
        tt = t_edges(kk);
        occ(kk) = any(tt >= it.segStart & tt < it.segEnd);
    end
    stairs(ax, t_edges, occ, 'Color', opt.lineColor, 'LineWidth', 1.8);

    % --- filled final rectangles on top ---
    for q = 1:numel(it.spaces)
        s = it.spaces(q);
        x = it.segStart(q);
        w = it.segEnd(q) - it.segStart(q);

        rectangle(ax, 'Position', [x, y0, w, h], ...
            'FaceColor', opt.spaceColors(s,:), ...
            'EdgeColor', 'none');
    end

    ylabel(ax, sprintf('V%d-K%d', it.veh, it.k), ...
        'Rotation', 0, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 12);

    ylim(ax, [0 1.2]);
    yticks(ax, [0 0.5 1]);
    xlim(ax, [xmin xmax]);

    if r < Nax
        set(ax, 'XTickLabel', []);
    end
end

% -------------------- space rows --------------------
for s = 1:Ns
    r = Nv + s;
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', fig, 'Position', [opt.marg_w bottom axW axH]);
    axes_list(r) = ax;

    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, ...
        'FontSize', 9, ...
        'Box', 'on', ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.16, ...
        'Layer', 'bottom');

    y0 = 0;
    h  = 1.0;

    % --- light-blue baseline first: only y=0 reference line ---
    stairs(ax, [xmin xmax], [0 0], 'Color', opt.lineColor, 'LineWidth', 1.8);

    % --- draw each occupying vehicle as one outlined block ---
    for ii = 1:Nv
        it = items(ii);
        idx_s = find(it.spaces == s);

        for q = idx_s
            x = it.segStart(q);
            w = it.segEnd(q) - it.segStart(q);

            rectangle(ax, 'Position', [x, y0, w, h], ...
                'FaceColor', 'none', ...
                'EdgeColor', opt.spaceColors(s,:), ...
                'LineWidth', 2.2);

            % label inside each block
            xc = x + w/2;
            text(ax, xc, 0.5, sprintf('V%d', it.veh), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'FontSize', 16, ...
                'Color', 'k', ...
                'Clipping', 'on');
        end
    end

    ylabel(ax, sprintf('S%d', s), ...
        'Rotation', 0, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 10);

    ylim(ax, [0 1.2]);
    yticks(ax, [0 0.5 1]);
    xlim(ax, [xmin xmax]);

    if s < Ns
        set(ax, 'XTickLabel', []);
    else
        xlabel(ax, 'Time (seconds)', 'FontSize', 12);
    end
end

sgtitle(fig, sprintf('Intersection %d', agent_i), ...
    'FontWeight', 'bold', 'FontSize', 12);

linkaxes(axes_list, 'x');

end

% =========================================================
function gamma_last = extract_final_gamma(NODES, Path_min)
lastNode = NODES{Path_min(end)};

if iscell(lastNode)
    if numel(lastNode) >= 11
        gamma_last = lastNode{11};
    else
        error('Cell node does not contain entry {11} for gamma.');
    end
elseif isstruct(lastNode)
    if isfield(lastNode, 'gamma')
        gamma_last = lastNode.gamma;
    elseif isfield(lastNode, 'Gamma')
        gamma_last = lastNode.Gamma;
    else
        error('Struct node does not contain field gamma/Gamma.');
    end
else
    error('Unsupported NODES format.');
end
end

% =========================================================
function g = fetch_vehicle_gamma(gamma_last, veh, localIdx)
g = [];

if iscell(gamma_last)
    if veh <= numel(gamma_last) && ~isempty(gamma_last{veh})
        g = gamma_last{veh};
    elseif localIdx <= numel(gamma_last) && ~isempty(gamma_last{localIdx})
        g = gamma_last{localIdx};
    end
elseif isnumeric(gamma_last) && isvector(gamma_last)
    if veh <= numel(gamma_last)
        g = gamma_last(veh);
    elseif localIdx <= numel(gamma_last)
        g = gamma_last(localIdx);
    end
end

if isempty(g)
    return;
end

if iscell(g)
    g = g{1};
end

g = g(:);
g = g(find(isfinite(g), 1, 'last'));
end