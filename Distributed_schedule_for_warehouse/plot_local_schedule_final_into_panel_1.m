function ax_list = plot_local_schedule_final_into_panel_1( ...
    parentPanel, NODES, Path_min, agent_i, valid_systems, const, varargin)
% Draw one intersection local schedule inside a given panel/uipanel.
% parentPanel can be a uipanel or figure.

p = inputParser;
addParameter(p, 'gap', 0.004);
addParameter(p, 'panelColor', 'none');
addParameter(p, 'axColor',  'none');
addParameter(p, 'title_pad', 0.05);
addParameter(p, 'showTitle', false);   % false = no panel title (label added in HTML)
addParameter(p, 'marg_h', 0.07); 
addParameter(p, 'marg_w', 0.10);
addParameter(p, 'lineColor', [0.35 0.75 0.95]);
addParameter(p, 'spaceColors', [ ...
    0.16 0.95 0.16;   % S1
    0.98 0.84 0.00;   % S2
    1.00 0.55 0.00;   % S3
    0.12 0.66 0.84;   % S4
    0.95 0.10 0.10]); % S5

% ===== new options =====
addParameter(p, 'x_prev', []);
addParameter(p, 'y_prev', []);
addParameter(p, 'showGenLine', true);
addParameter(p, 'showDelayPatch', true);
addParameter(p, 'Dt_override', []);
addParameter(p, 'genLineColor', [0.55 0.10 0.75]);   % purple
addParameter(p, 'delayColor', [0.85 0.85 0.85]);     % light gray
addParameter(p, 'delayAlpha', 0.55);
addParameter(p, 'genLineWidth', 1.6);
addParameter(p, 'delayBarHeight', 1.0); % same height as task bar
parse(p, varargin{:});
opt = p.Results;

set(parentPanel, 'BackgroundColor', opt.panelColor);

IntSpaceDB = const.IntSpaceDB;
pathInfo = const.pathInfo;
pathInfo_agent_chain = const.pathInfo_agent_chain;
alpha_tilde = const.alpha_tilde;

if ~isempty(opt.Dt_override)
    Dt = opt.Dt_override;
elseif isfield(const, 'Dt')
    Dt = const.Dt;
else
    error('Dt not found. Provide const.Dt or pass ''Dt_override''.');
end

x_prev = opt.x_prev;
y_prev = opt.y_prev; %#ok<NASGU> % reserved if later needed

numSpaces = IntSpaceDB{agent_i}.numSpaces;
kn = 1;

gamma_last = extract_final_gamma(NODES, Path_min);

items = struct('veh', {}, 'k', {}, 'routeId', {}, ...
               'spaces', {}, 'dur', {}, ...
               't0', {}, 'tf', {}, ...
               'segStart', {}, 'segEnd', {}, ...
               'genTime', {}, 'delayStart', {}, 'delayEnd', {});

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

    %% ===== generation time for visualization =====
genTime = NaN;
xroad = NaN;
prev_road = NaN;

if posi == 1
    % first intersection in chain
    if iscell(alpha_tilde{n})
        genTime = alpha_tilde{n}{kn};
    else
        genTime = alpha_tilde{n}(kn);
    end
else
    % chain = [int, road, int, road, ...]
    prev_road = chain(posi - 1);

    if ~isempty(x_prev) && numel(x_prev) >= prev_road && ...
            ~isempty(x_prev{prev_road}) && numel(x_prev{prev_road}) >= n && ...
            ~isempty(x_prev{prev_road}{n})

        xroad = x_prev{prev_road}{n};

        if iscell(xroad)
            xroad = xroad{kn};
        elseif numel(xroad) >= kn
            xroad = xroad(kn);
        end

        genTime = xroad + Dt;
    end
end

% fprintf('\nagent %d, veh %d:\n', agent_i, n);
% fprintf('  posi    = %d\n', posi);
% fprintf('  t0      = %.4f\n', t0);
% fprintf('  tf      = %.4f\n', tf);
% fprintf('  genTime = %.4f\n', genTime);
% fprintf('  delay   = %.4f\n', t0 - genTime);
% 
% if posi == 1
%     fprintf('  alpha   = %.4f\n', genTime);
% else
%     fprintf('  prev_road = %d\n', prev_road);
%     fprintf('  xroad     = %.4f\n', xroad);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % delay patch range
    delayStart = genTime;
    delayEnd   = t0;

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
        'delayStart', delayStart, ...
        'delayEnd', delayEnd); %#ok<AGROW>
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

% x-limits should include generation time too
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

% panel title (suppressed by default — label added as HTML overlay instead)
if opt.showTitle
    title_h = 0.05;
    title_y = 0.94;
    uicontrol('Parent', parentPanel, 'Style', 'text', ...
        'String', sprintf('Merging Zone %d', agent_i), ...
        'Units', 'normalized', ...
        'Position', [0.25 title_y 0.50 title_h], ...
        'BackgroundColor', opt.panelColor, ...
        'FontWeight', 'bold', 'FontSize', 14);
end

% =========================
% vehicle rows
% =========================
for r = 1:Nv
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, 'FontSize', 16, 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.16, 'Layer', 'bottom');

    y0 = 0; h = 1.0;
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

    % ---- delay gray patch: from generation time to actual local task start ----
if opt.showDelayPatch && isfinite(it.delayStart) && isfinite(it.delayEnd) ...
        && (it.delayEnd > it.delayStart)

    draw_delay_hatch(ax, ...
        it.delayStart, it.delayEnd, ...
        y0, y0 + opt.delayBarHeight, ...
        [0.75 0.75 0.75], ...   % hatch color
        0.18);                  % spacing
end
 
    % ---- actual local task colored segments ----
    for q = 1:numel(it.spaces)
        s = it.spaces(q);
        x = it.segStart(q);
        w = it.segEnd(q) - it.segStart(q);
        rectangle(ax, 'Position', [x, y0, w, h], ...
            'FaceColor', opt.spaceColors(s,:), 'EdgeColor', 'none');
    end  

    % ---- purple dashed generation line ----
    if opt.showGenLine && isfinite(it.genTime)
        xline(ax, it.genTime, '--', ...
            'Color', opt.genLineColor, ...
            'LineWidth', opt.genLineWidth);
    end

    text(ax, xmin - 0.005*(xmax-xmin), 0.5, sprintf('N%d-K%d', it.veh, it.k), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
        'FontSize', 16, 'Clipping', 'off');

    ylim(ax, [0 1.2]);
    yticks(ax, []);
    xlim(ax, [xmin xmax]);

    if r < Nax
        set(ax, 'XTickLabel', []);
    end
end

% =========================
% space rows
% =========================
for s = 1:Ns
    r = Nv + s;
    bottom = 1 - opt.marg_h - opt.title_pad - r*axH - (r-1)*opt.gap;
    ax = axes('Parent', parentPanel, 'Position', [opt.marg_w bottom axW axH]);
    ax_list(r) = ax;
    hold(ax, 'on');
    set(ax, 'Color', opt.axColor, 'FontSize', 16, 'Box', 'on', ...
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

            text(ax, x + w/2, 0.5, sprintf('N%d', it.veh), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'FontSize', 16, ...
                'Color', 'k', ...
                'Clipping', 'on');
        end
    end

    text(ax, xmin - 0.001*(xmax-xmin), 0.5, sprintf('M%d', s), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
        'FontSize', 16, 'Clipping', 'off');

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

function draw_delay_hatch(ax, x1, x2, y1, y2, lineColor, spacing)
% Draw diagonal hatch lines inside rectangle [x1,x2] x [y1,y2]

if x2 <= x1 || y2 <= y1
    return;
end

hold(ax, 'on');

H = y2 - y1;
W = x2 - x1;

% 画斜率为 +1 的线段族：y = (x-c) + y1
c_min = x1 - H;
c_max = x2;

c_vals = c_min:spacing:c_max;

for c = c_vals
    % line segment intersection with rectangle
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

% 可选：外框淡淡画一下
plot(ax, [x1 x2 x2 x1 x1], [y1 y1 y2 y2 y1], '-', ...
    'Color', [0.82 0.82 0.82], 'LineWidth', 0.5);
end