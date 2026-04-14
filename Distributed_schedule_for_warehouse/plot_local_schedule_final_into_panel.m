function ax_list = plot_local_schedule_final_into_panel( ...
    parentPanel, NODES, Path_min, agent_i, valid_systems, const, varargin)
% Draw one intersection local schedule inside a given panel/uipanel.
% parentPanel can be a uipanel or figure.

p = inputParser;
addParameter(p, 'gap', 0.004);
addParameter(p, 'panelColor', [0.98 0.98 0.98]);
addParameter(p, 'axColor',  [0.98 0.98 0.98]);
addParameter(p, 'title_pad', 0.05);
addParameter(p, 'marg_h', 0.07);
addParameter(p, 'marg_w', 0.10);
addParameter(p, 'lineColor', [0.35 0.75 0.95]);
addParameter(p, 'spaceColors', [ ...
    0.16 0.95 0.16;   % S1
    0.98 0.84 0.00;   % S2
    1.00 0.55 0.00;   % S3
    0.12 0.66 0.84;   % S4
    0.95 0.10 0.10]); % S5
parse(p, varargin{:});
opt = p.Results;

set(parentPanel, 'BackgroundColor', opt.panelColor);

IntSpaceDB = const.IntSpaceDB;
pathInfo = const.pathInfo;
pathInfo_agent_chain = const.pathInfo_agent_chain;

numSpaces = IntSpaceDB{agent_i}.numSpaces;
kn = 1;

gamma_last = extract_final_gamma(NODES, Path_min);

items = struct('veh', {}, 'k', {}, 'routeId', {}, ...
               'spaces', {}, 'dur', {}, ...
               't0', {}, 'tf', {}, ...
               'segStart', {}, 'segEnd', {});

for idx = 1:numel(valid_systems)
    n = valid_systems(idx);

    chain = pathInfo_agent_chain{n}{kn};
    posi = find(chain == agent_i, 1, 'first');
    if isempty(posi), continue; end

    pos_int = (posi + 1) / 2;
    routeId = pathInfo{n}.routeId(pos_int);

    space_seq = IntSpaceDB{agent_i}.routeSpace{routeId};
    dur_seq   = IntSpaceDB{agent_i}.routeDur{routeId};
    if isempty(space_seq) || isempty(dur_seq), continue; end

    tf = fetch_vehicle_gamma(gamma_last, n, idx);
    if isempty(tf) || ~isfinite(tf), continue; end

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
    ax_list = gobjects(0);
    return;
end

[~, ord] = sort([items.veh]);
items = items(ord);

Nv  = numel(items);
Ns  = numSpaces;
Nax = Nv + Ns;

axH = (1 - 2*opt.marg_h - (Nax-1)*opt.gap - opt.title_pad) / Nax;
axW = 1 - 2*opt.marg_w;
ax_list = gobjects(Nax,1);

xmin = min([items.t0]) - 0.4;
xmax = max([items.tf]) + 0.4;

% panel title
title_h = 0.05;
title_y = 0.94;

% panel title
uicontrol('Parent', parentPanel, 'Style', 'text', ...
    'String', sprintf('Intersection %d', agent_i), ...
    'Units', 'normalized', ...
    'Position', [0.25 title_y 0.50 title_h], ...
    'BackgroundColor', opt.panelColor, ...
    'FontWeight', 'bold', 'FontSize', 14); %[0.30 1-opt.marg_h+0.005 0.40 0.04], ...

% vehicle rows
for r = 1:Nv
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, 'FontSize', 12, 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.16, 'Layer', 'bottom');

    y0 = 0; h = 1.0;
    it = items(r);

    t_edges = [xmin, sort([it.segStart, it.segEnd]), xmax];
    t_edges = unique(t_edges);
    occ = zeros(size(t_edges));
    for kk = 1:numel(t_edges)
        tt = t_edges(kk);
        occ(kk) = any(tt >= it.segStart & tt < it.segEnd);
    end
    stairs(ax, t_edges, occ, 'Color', opt.lineColor, 'LineWidth', 1.5);

    for q = 1:numel(it.spaces)
        s = it.spaces(q);
        x = it.segStart(q);
        w = it.segEnd(q) - it.segStart(q);
        rectangle(ax, 'Position', [x, y0, w, h], ...
            'FaceColor', opt.spaceColors(s,:), 'EdgeColor', 'none');
    end

    text(ax, xmin - 0.06*(xmax-xmin), 0.5, sprintf('V%d-K%d', it.veh, it.k), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
        'FontSize', 12, 'Clipping', 'off');

    ylim(ax, [0 1.2]); 
    yticks(ax, []); 
    xlim(ax, [xmin xmax]);

    if r < Nax
        set(ax, 'XTickLabel', []);
    end
end

% space rows
for s = 1:Ns
    r = Nv + s;
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, 'FontSize', 12, 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.16, 'Layer', 'bottom');

    y0 = 0; h = 1.0;

    stairs(ax, [xmin xmax], [0 0], 'Color', opt.lineColor, 'LineWidth', 1.5);

    for ii = 1:Nv
        it = items(ii);
        idx_s = find(it.spaces == s);
        for q = idx_s
            x = it.segStart(q);
            w = it.segEnd(q) - it.segStart(q);

            rectangle(ax, 'Position', [x, y0, w, h], ...
                'FaceColor', 'none', ...
                'EdgeColor', opt.spaceColors(s,:), ...
                'LineWidth', 2.0);

            text(ax, x + w/2, 0.5, sprintf('V%d', it.veh), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'FontSize', 16, ...
                'Color', 'k', ...
                'Clipping', 'on');
        end
    end

    text(ax, xmin - 0.06*(xmax-xmin), 0.5, sprintf('S%d', s), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
        'FontSize', 12, 'Clipping', 'off');

    ylim(ax, [0 1.2]);
    yticks(ax, []);
    xlim(ax, [xmin xmax]);

    if s < Ns
        set(ax, 'XTickLabel', []);
    else
        xlabel(ax, 'Time (seconds)', 'FontSize', 12);
    end
end

linkaxes(ax_list, 'x');

end

function gamma_last = extract_final_gamma(NODES, Path_min)
lastNode = NODES{Path_min(end)};
if iscell(lastNode)
    gamma_last = lastNode{11};
elseif isstruct(lastNode)
    if isfield(lastNode,'gamma')
        gamma_last = lastNode.gamma;
    else
        gamma_last = lastNode.Gamma;
    end
else
    error('Unsupported NODES format.');
end
end

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
if isempty(g), return; end
if iscell(g), g = g{1}; end
g = g(:);
g = g(find(isfinite(g),1,'last'));
end