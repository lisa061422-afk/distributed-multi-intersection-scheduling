function plot_C_ADMM2_5int(residual_r, residual_s, delay_costs, x_prev, y_prev, pathInfo_agent_chain, N, k)
% plot_C_ADMM2_5int
% Plotting function for 5-intersection C-ADMM traffic scheduling.
%
% Agent layout (5 intersections):
%   Agents 1..5  : intersections
%   Agents 6..10 : roads
%   Agent  11    : terminal

if nargin < 8 || isempty(k)
    k = min([numel(residual_r), numel(residual_s), numel(delay_costs)]);
end

kn = 1;

% ========================= Style =========================
FS_ax  = 20;
FS_tit = 18;
FS_txt = 16;

LW_ax  = 1.0;
LW_bar = 1.0;

barH = 0.72;

useGrayAxes = false;
showBarText_intersection = true;
showBarText_road = true;

colors = lines(max(N,1));
colors = 0.7*colors + 0.3*ones(size(colors));
if N >= 8
    colors(8,:) = 0.5*colors(8,:) + 0.5*[1 1 1];
end

NUM_INT  = 5;
NUM_ROAD = 5;   % roads 6..10
TERMINAL = 11;

% =========================================================
% Figure 1: residuals
% =========================================================
figure('Color','w','Units','pixels','Position',[80 120 780 520]);
ax = axes('Position',[0.12 0.14 0.82 0.76]);
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
ax.FontSize = FS_ax; ax.LineWidth = LW_ax;

semilogy(ax, residual_r(1:k), 'b-', 'LineWidth', 2, 'DisplayName', 'Primal Residual');
semilogy(ax, residual_s(1:k), 'r--', 'LineWidth', 2, 'DisplayName', 'Dual Residual');

xlabel(ax,'Iteration','FontSize',FS_ax);
ylabel(ax,'Residual (log scale)','FontSize',FS_ax);
title(ax,'ADMM Residuals','FontSize',FS_tit);
legend(ax,'show','Location','best');

% =========================================================
% Figure 2: delay cost
% =========================================================
figure('Color','w','Units','pixels','Position',[120 140 780 520]);
ax = axes('Position',[0.12 0.14 0.82 0.76]);
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
ax.FontSize = FS_ax; ax.LineWidth = LW_ax;

plot(ax, 1:k, delay_costs(1:k), 'b-o', 'LineWidth', 2, 'MarkerSize', 5);

xlabel(ax,'Iteration','FontSize',FS_ax);
ylabel(ax,'Total Time Delay Cost','FontSize',FS_ax);
title(ax,'Delay Cost over Iterations','FontSize',FS_tit);

% =========================================================
% Figure 3: 5 intersections (3-top, 2-bottom layout)
% =========================================================
figure('Color','w','Units','pixels','Position',[80 80 1500 900]);

leftMargin   = 0.08;
rightMargin  = 0.03;
topMargin    = 0.06;
bottomMargin = 0.08;
gapY         = 0.04;

% Collect data for all 5 intersections
intData = repmat(struct('usedVeh',[],'posOrder',[],'stList',[],'enList',[],'nLocal',1), 1, NUM_INT);
for a = 1:NUM_INT
    [usedVeh, posOrder, stList, enList] = collect_used_vehicles_for_agent(a, x_prev, y_prev, pathInfo_agent_chain, N, kn);
    intData(a).usedVeh  = usedVeh;
    intData(a).posOrder = posOrder;
    intData(a).stList   = stList;
    intData(a).enList   = enList;
    intData(a).nLocal   = max(numel(usedVeh),1);
end

[tmin_int, tmax_int] = compute_global_xlim_from_data(intData);
rowCounts = [intData.nLocal];
totalRows = sum(rowCounts);

figW = 1 - leftMargin - rightMargin;
figH = 1 - topMargin - bottomMargin - (NUM_INT-1)*gapY;
currentTop = 1 - topMargin;

for a = 1:NUM_INT
    h = figH * rowCounts(a) / totalRows;
    bottom = currentTop - h;

    ax = axes('Position',[leftMargin, bottom, figW, h]); hold(ax,'on');
    grid(ax,'on'); box(ax,'on');
    ax.FontSize = FS_ax; ax.LineWidth = LW_ax;
    if useGrayAxes, ax.Color = [0.98 0.98 0.98]; end

    usedVeh  = intData(a).usedVeh;
    posOrder = intData(a).posOrder;
    stList   = intData(a).stList;
    enList   = intData(a).enList;
    nLocal   = intData(a).nLocal;

    if isempty(usedVeh)
        xlim(ax,[tmin_int tmax_int]);
        ylim(ax,[0.5 1.5]);
        yticks(ax,1);
        yticklabels(ax,{'-'});
    else
        for j = 1:numel(usedVeh)
            vehID = usedVeh(j);
            st    = stList(j);
            en    = enList(j);
            posK  = posOrder(j);
            localRow = nLocal - j + 1;
            draw_bar(ax, st, en, localRow, vehID, posK, colors, barH, FS_txt, LW_bar, showBarText_intersection);
        end

        xlim(ax,[tmin_int tmax_int]);
        ylim(ax,[0.5 nLocal+0.5]);
        yticks(ax,1:nLocal);
        yticklabels(ax, flip(arrayfun(@(v) sprintf('N%d', v), usedVeh, 'UniformOutput', false)));
    end

    title(ax, sprintf('Intersection %d', a), 'FontSize', FS_tit);

    if a < NUM_INT
        ax.XTickLabel = [];
    else
        xlabel(ax,'Time (seconds)','FontSize',FS_ax);
    end

    currentTop = bottom - gapY;
end

% =========================================================
% Figure 4: roads 6..10 and terminal 11
% =========================================================
figure('Color','w','Units','pixels','Position',[120 100 1500 900]);

agentNames = {'Road 1','Road 2','Road 3','Road 4','Road 5','Terminal'};
leftMargin   = 0.08;
rightMargin  = 0.03;
topMargin    = 0.06;
bottomMargin = 0.08;
gapY         = 0.018;

numPanels = NUM_ROAD + 1;  % 5 roads + 1 terminal = 6
roadData = repmat(struct('usedVeh',[],'posOrder',[],'stList',[],'enList',[],'nLocal',1), 1, numPanels);
for idx = 1:numPanels
    if idx <= NUM_ROAD
        a = NUM_INT + idx;   % roads 6..10
    else
        a = TERMINAL;        % terminal 11
    end
    [usedVeh, posOrder, stList, enList] = collect_used_vehicles_for_agent(a, x_prev, y_prev, pathInfo_agent_chain, N, kn);
    roadData(idx).usedVeh  = usedVeh;
    roadData(idx).posOrder = posOrder;
    roadData(idx).stList   = stList;
    roadData(idx).enList   = enList;
    roadData(idx).nLocal   = max(numel(usedVeh),1);
end

[tmin_road, tmax_road] = compute_global_xlim_from_data(roadData);
rowCounts2 = [roadData.nLocal];
totalRows2 = sum(rowCounts2);

figW = 1 - leftMargin - rightMargin;
figH = 1 - topMargin - bottomMargin - (numPanels-1)*gapY;
currentTop = 1 - topMargin;

for idx = 1:numPanels
    h = figH * rowCounts2(idx) / totalRows2;
    bottom = currentTop - h;

    ax = axes('Position',[leftMargin, bottom, figW, h]); hold(ax,'on');
    grid(ax,'on'); box(ax,'on');
    ax.FontSize = FS_ax; ax.LineWidth = LW_ax;
    if useGrayAxes, ax.Color = [0.98 0.98 0.98]; end

    usedVeh  = roadData(idx).usedVeh;
    posOrder = roadData(idx).posOrder;
    stList   = roadData(idx).stList;
    enList   = roadData(idx).enList;
    nLocal   = roadData(idx).nLocal;

    if isempty(usedVeh)
        xlim(ax,[tmin_road tmax_road]);
        ylim(ax,[0.5 1.5]);
        yticks(ax,1);
        yticklabels(ax,{'-'});
    else
        for j = 1:numel(usedVeh)
            vehID = usedVeh(j);
            st    = stList(j);
            en    = enList(j);
            posK  = posOrder(j);
            localRow = nLocal - j + 1;
            draw_bar(ax, st, en, localRow, vehID, posK, colors, barH, FS_txt, LW_bar, showBarText_road);
        end

        xlim(ax,[tmin_road tmax_road]);
        ylim(ax,[0.5 nLocal+0.5]);
        yticks(ax,1:nLocal);
        yticklabels(ax, flip(arrayfun(@(v) sprintf('N%d', v), usedVeh, 'UniformOutput', false)));
    end

    title(ax, agentNames{idx}, 'FontSize', FS_tit);

    if idx < numPanels
        ax.XTickLabel = [];
    else
        xlabel(ax,'Time (seconds)','FontSize',FS_ax);
    end

    currentTop = bottom - gapY;
end

end

% =========================================================
% Collect vehicles that actually appear on agent a
% =========================================================
function [usedVeh, posOrder, stList, enList] = collect_used_vehicles_for_agent(a, x_prev, y_prev, pathInfo_agent_chain, N, kn)

usedVeh  = [];
posOrder = [];
stList   = [];
enList   = [];

for n = 1:N
    if isempty(pathInfo_agent_chain{n}) || numel(pathInfo_agent_chain{n}) < kn
        continue;
    end

    chain = pathInfo_agent_chain{n}{kn};
    posi = find(chain == a, 1, 'first');
    if isempty(posi)
        continue;
    end

    posK = ceil(posi/2);

    % x start
    if a > numel(x_prev) || isempty(x_prev{a}) || isempty(x_prev{a}{n}) || numel(x_prev{a}{n}) < kn
        continue;
    end
    st = x_prev{a}{n}(kn);
    if isnan(st)
        continue;
    end

    % y end
    NUM_NON_TERMINAL = numel(x_prev) - 1;  % last non-terminal agent
    if a <= NUM_NON_TERMINAL
        if a > numel(y_prev) || isempty(y_prev{a}) || isempty(y_prev{a}{n}) || numel(y_prev{a}{n}) < kn
            continue;
        end
        en = y_prev{a}{n}(kn);
        if isnan(en) || en <= st
            continue;
        end
    else
        % terminal agent: create a short visible bar
        en = st + 0.08;
    end

    usedVeh(end+1)  = n; %#ok<AGROW>
    posOrder(end+1) = posK; %#ok<AGROW>
    stList(end+1)   = st; %#ok<AGROW>
    enList(end+1)   = en; %#ok<AGROW>
end
end

% =========================================================
% Compute global x limits from collected data
% =========================================================
function [t_min, t_max] = compute_global_xlim_from_data(data)

allStart = [];
allEnd   = [];

for i = 1:numel(data)
    allStart = [allStart, data(i).stList]; %#ok<AGROW>
    allEnd   = [allEnd,   data(i).enList]; %#ok<AGROW>
end

if isempty(allStart) || isempty(allEnd)
    t_min = 0;
    t_max = 10;
    return;
end

t_min = min(allStart);
t_max = max(allEnd);

pad = max(0.2, 0.02*(t_max - t_min));
t_min = max(0, t_min - pad);
t_max = t_max + pad;
end

% =========================================================
% Draw one bar
% =========================================================
function draw_bar(ax, st, en, rowIdx, vehID, posK, colors, barH, FS_txt, LW_bar, useText)

dur = en - st;
y0  = rowIdx - barH/2;

rectangle(ax, ...
    'Position', [st, y0, dur, barH], ...
    'FaceColor', colors(vehID,:), ...
    'EdgeColor', [0.53 0.81 0.92], ...
    'LineWidth', LW_bar);

if useText
    text(ax, st + dur/2, rowIdx, sprintf('N%d-K%d', vehID, posK), ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','middle', ...
        'FontSize',FS_txt, ...
        'Color','k', ...
        'HitTest','off');
end
end
