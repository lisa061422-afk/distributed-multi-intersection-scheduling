function ax_list = plot_local_schedule_final_into_panel_centralized( ...
    parentPanel, DATA, const, agent_i, valid_systems, varargin)
%PLOT_LOCAL_SCHEDULE_FINAL_INTO_PANEL_CENTRALIZED
% Centralized version of one-intersection local final schedule plot.
%
% Inputs:
%   parentPanel   : uipanel or figure handle
%   DATA          : output from build_centralized_schedule_data(...)
%   const         : main const struct
%   agent_i       : intersection index
%   valid_systems : systems passing this intersection
%
% Display style aligned with plot_local_schedule_final_into_panel_1.

p = inputParser;
addParameter(p, 'gap', 0.004);
addParameter(p, 'panelColor', 'none');
addParameter(p, 'axColor',  'none');
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
addParameter(p, 'showGenLine', true);
addParameter(p, 'showDelayPatch', true);
% addParameter(p, 'genLineColor', [0.55 0.10 0.75]);   % purple
addParameter(p, 'genLineColor', 'b');
addParameter(p, 'delayColor', [0.85 0.85 0.85]);
addParameter(p, 'delayAlpha', 0.55);
addParameter(p, 'genLineWidth', 1.6);
addParameter(p, 'delayBarHeight', 1.0);
parse(p, varargin{:});
opt = p.Results;

set(parentPanel, 'BackgroundColor', opt.panelColor);

Veh = const.Veh;
IntSpaceDB = const.IntSpaceDB;
numSpaces = IntSpaceDB{agent_i}.numSpaces;

items = struct('veh', {}, 'k', {}, 'routeId', {}, ...
               'spaces', {}, 'dur', {}, ...
               't0', {}, 'tf', {}, ...
               'segStart', {}, 'segEnd', {}, ...
               'genTime', {}, 'delayStart', {}, 'delayEnd', {});

% ============================================================
% Build plotting items
% ============================================================
for idx = 1:numel(valid_systems)
    n = valid_systems(idx);

    intSeq = Veh(n).intSeq;
    pos_int = find(intSeq == agent_i, 1, 'first');
    if isempty(pos_int)
        continue;
    end

    routeId = Veh(n).routeId(pos_int);

    if routeId < 1 || routeId > numel(IntSpaceDB{agent_i}.routeSpace)
        continue;
    end

    space_seq = IntSpaceDB{agent_i}.routeSpace{routeId};
    dur_seq   = IntSpaceDB{agent_i}.routeDur{routeId};

    if isempty(space_seq) || isempty(dur_seq)
        continue;
    end

    tf = DATA.gamma_local{agent_i}(n);
    genTime = DATA.alpha_local{agent_i}(n);

    if isempty(tf) || isempty(genTime) || ~isfinite(tf) || ~isfinite(genTime)
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
        'segEnd', segEnd, ...
        'genTime', genTime, ...
        'delayStart', genTime, ...
        'delayEnd', t0); %#ok<AGROW>
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

% ============================================================
% x-limits
% ============================================================
all_t_min = [items.t0];
all_t_max = [items.tf];
all_gen   = [items.genTime];
all_gen   = all_gen(isfinite(all_gen));

if isempty(all_gen)
    xmin = min(all_t_min) - 0.4;
else
    xmin = min([all_t_min, all_gen]) - 0.4;
end
xmax = max(all_t_max) + 0.4;

% ============================================================
% Title
% ============================================================
title_h = 0.05;
title_y = 0.94;

uicontrol('Parent', parentPanel, 'Style', 'text', ...
    'String', sprintf('Intersection %d', agent_i), ...
    'Units', 'normalized', ...
    'Position', [0.25 title_y 0.50 title_h], ...
    'BackgroundColor', opt.panelColor, ...
    'FontWeight', 'bold', 'FontSize', 16);

% ============================================================
% Vehicle rows
% ============================================================
for r = 1:Nv
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');

    set(ax, 'Color', opt.axColor, 'FontSize', 16, 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.16, 'Layer', 'bottom');

    y0 = 0; 
    h  = 1.0;
    it = items(r);

    % baseline occupancy line
    t_edges = [xmin, sort([it.segStart, it.segEnd]), xmax];
    t_edges = unique(t_edges);
    occ = zeros(size(t_edges));
    for kk = 1:numel(t_edges)
        tt = t_edges(kk);
        occ(kk) = any(tt >= it.segStart & tt < it.segEnd);
    end
    stairs(ax, t_edges, occ, 'Color', opt.lineColor, 'LineWidth', 1.5);

    % delay hatch
    if opt.showDelayPatch && isfinite(it.delayStart) && isfinite(it.delayEnd) ...
            && (it.delayEnd > it.delayStart)

        draw_delay_hatch(ax, ...
            it.delayStart, it.delayEnd, ...
            y0, y0 + opt.delayBarHeight, ...
            [0.75 0.75 0.75], ...
            0.18);
    end

    % actual local task colored segments
    for q = 1:numel(it.spaces)
        s = it.spaces(q);
        x = it.segStart(q);
        w = it.segEnd(q) - it.segStart(q);

        rectangle(ax, 'Position', [x, y0, w, h], ...
            'FaceColor', opt.spaceColors(s,:), ...
            'EdgeColor', 'none');
    end

    % generation line
    if opt.showGenLine && isfinite(it.genTime)
        xline(ax, it.genTime, '--', ...
            'Color', opt.genLineColor, ...
            'LineWidth', opt.genLineWidth);
    end

    text(ax, xmin - 0.005*(xmax-xmin), 0.5, sprintf('N%d-K%d', it.veh, it.k), ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 16, ...
        'Clipping', 'off');

    ylim(ax, [0 1.2]);
    yticks(ax, []);
    xlim(ax, [xmin xmax]);

    if r < Nax
        set(ax, 'XTickLabel', []);
    end
end

% ============================================================
% Space rows
% ============================================================
for s = 1:Ns
    r = Nv + s;
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');

    set(ax, 'Color', opt.axColor, 'FontSize', 16, 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.16, 'Layer', 'bottom');

    y0 = 0;
    h  = 1.0;

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

            text(ax, x + w/2, 0.5, sprintf('N%d', it.veh), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'FontSize', 16, ...
                'Color', 'k', ...
                'Clipping', 'on');
        end
    end

    text(ax, xmin - 0.001*(xmax-xmin), 0.5, sprintf('M%d', s), ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 16, ...
        'Clipping', 'off');

    ylim(ax, [0 1.2]);
    yticks(ax, []);
    xlim(ax, [xmin xmax]);

    if s < Ns
        set(ax, 'XTickLabel', []);
    else
        xlabel(ax, 'Time (seconds)', 'FontSize', 16);
    end
end

linkaxes(ax_list, 'x');

end

function draw_delay_hatch(ax, x1, x2, y1, y2, lineColor, spacing)
% Draw diagonal hatch lines inside rectangle [x1,x2] x [y1,y2]

if x2 <= x1 || y2 <= y1
    return;
end

hold(ax, 'on');

H = y2 - y1;
c_min = x1 - H;
c_max = x2;
c_vals = c_min:spacing:c_max;

for c = c_vals
    xa = max(x1, c);
    xb = min(x2, c + H);

    if xb > xa
        ya = y1 + (xa - c);
        yb = y1 + (xb - c);

        plot(ax, [xa xb], [ya yb], '-', ...
            'Color', lineColor, ...
            'LineWidth', 0.8, ...
            'Clipping', 'on');
    end
end

plot(ax, [x1 x2 x2 x1 x1], [y1 y1 y2 y2 y1], '-', ...
    'Color', [0.82 0.82 0.82], 'LineWidth', 0.5);
end