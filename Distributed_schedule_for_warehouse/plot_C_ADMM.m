function plot_C_ADMM(residual_r, residual_s, delay_costs, x_prev, y_prev, pathInfo_agent_chain, N, k)
% plot_ADMM_results
% Plot residuals, delay cost, and Gantt charts for agents 1-9 (traffic C-ADMM).
%
% Inputs:
%   residual_r, residual_s : vectors (length >= k)
%   delay_costs            : vector (length >= k)
%   x_prev                 : 1x9 cell, each x_prev{a}{n}(kn) is scalar
%   y_prev                 : 1x8 cell, each y_prev{a}{n}(kn) is scalar
%   pathInfo_agent_chain   : 1xN cell, each pathInfo_agent_chain{n}{kn} is chain vector
%   N                      : number of systems/vehicles
%   k                      : last iteration index used for plotting
%
% Notes (your requested plotting behavior):
% - Keep your original subplot style (stable, no weird label drifting)
% - Bigger fonts, thicker bars
% - Subplots closer together
% - No useless whitespace below the last vehicle (VN sits at bottom)
% - Bar text shows the order index of the intersection along the vehicle path: "%d-IN"
%
% If you always want only one segment per vehicle, keep kn = 1.
% If later you want multiple kn segments, you can re-enable the kn-loop.

if nargin < 8 || isempty(k)
    k = min([numel(residual_r), numel(residual_s), numel(delay_costs)]);
end

% =================== Global style knobs ===================
FS_ax   = 18;     % axes tick/label font
FS_tit  = 18;     % axes title font
FS_sgt  = 18;     % sgtitle font
FS_txt  = 18;     % text inside bars
LW_ax   = 1.2;    % axes line width
LW_bar  = 1.0;    % rectangle edge width
barH    = 0.80;   % rectangle height (0~1)
gap     = 0.028;  % vertical gap between subplots (smaller = closer)
marg_h  = 0.070;  % top/bottom margin
marg_w  = 0.060;  % left/right margin
useGrayAxes = false; % true -> light gray axes background

% If you only want one bar per vehicle (as you said), fix kn=1.
kn = 1;

% =================== Figure 1: Residuals ===================
figure('Color','w','Units','normalized','Position',[0.08 0.18 0.55 0.55]);
ax = gca; ax.FontSize = FS_ax; ax.LineWidth = LW_ax;
semilogy(residual_r(1:k), 'b-', 'DisplayName', 'Primal Residual', 'LineWidth', 2); hold on;
semilogy(residual_s(1:k), 'r--', 'DisplayName', 'Dual Residual', 'LineWidth', 2);
xlabel('Iteration','FontSize',FS_ax);
ylabel('Residual (log scale)','FontSize',FS_ax);
title('ADMM Residuals over Iterations','FontSize',FS_tit);
legend('show','Location','best');
grid on; box on;

% =================== Figure 2: Total delay cost ===================
figure('Color','w','Units','normalized','Position',[0.08 0.18 0.55 0.55]);
ax = gca; ax.FontSize = FS_ax; ax.LineWidth = LW_ax;
plot(1:k, delay_costs(1:k), 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
xlabel('Iteration','FontSize',FS_ax);
ylabel('Total Time Delay Cost','FontSize',FS_ax);
title('Delay Cost over ADMM Iterations','FontSize',FS_tit);
grid on; box on;

% =================== Figure 3: Intersection Agents (1-4) ===================
figure('Color','w','Units','normalized','Position',[0.05 0.08 0.92 0.84]);

colors = lines(max(N,1));
agent_names = {'IN1', 'IN2', 'IN3', 'IN4'};
[t_min, t_max] = compute_xlim(x_prev, y_prev, pathInfo_agent_chain, 1, 4, N, kn);
% x_max = compute_xlim(x_prev, y_prev, 1, 4, N);

for a = 1:4
    ax = subplot(4,1,a); hold(ax,'on');

    % Style
    ax.FontSize  = FS_ax;
    ax.LineWidth = LW_ax;
    grid(ax,'on'); box(ax,'on');
    if useGrayAxes, ax.Color = [0.98 0.98 0.98]; end

    % Bars
    for n = 1:N
        if isempty(pathInfo_agent_chain{n}) || numel(pathInfo_agent_chain{n}) < kn, continue; end
        chain = pathInfo_agent_chain{n}{kn};

        posi = find(chain == a, 1, 'first');
        if isempty(posi), continue; end
        pos_int = ceil(posi/2); % order index along the vehicle path

        st = x_prev{a}{n}(kn);
        en = y_prev{a}{n}(kn);
        if isnan(st) || isnan(en) || en <= st, continue; end

        draw_bar(ax, st, en, n, N, colors, pos_int, barH, FS_txt, LW_bar);
    end

    % xlim(ax,[0 x_max]);
    xlim(ax,[t_min t_max]);
    ylim(ax,[0.5, N+1
        ]);     % tight: VN sits at bottom
    yticks(ax,1:N);
    yticklabels(ax, flip(arrayfun(@(nn) ['V', num2str(nn)], 1:N, 'UniformOutput', false)));

    % title(ax, ['Agent: ', agent_names{a}], 'FontSize',FS_tit);
    title(ax, sprintf('Intersection %d', a), 'FontSize', FS_tit);
    % Only show x tick labels on the last subplot
    if a ~= 4
        ax.XTickLabel = [];
    else
        xlabel(ax,'Time(seconds)','FontSize',FS_ax);
    end
end
% sgtitle('Intersection Agents (1–4) Gantt Chart','FontSize',FS_sgt);

% make subplots closer
tight_subplots(gcf, 4, 1, gap, marg_h, marg_w);

% =================== Figure 4: Road Agents (5-8) and Terminal (9) ===================
figure('Color','w','Units','normalized','Position',[0.05 0.06 0.92 0.88]);

agent_names2 = {'RD1', 'RD2', 'RD3', 'RD4', 'x9'};
[t_min, t_max] = compute_xlim(x_prev, y_prev, pathInfo_agent_chain, 5, 9, N, kn);
% x_max = compute_xlim(x_prev, y_prev, 5, 9, N);

for idx = 1:5
    a = idx + 4;  % 5..9
    ax = subplot(5,1,idx); hold(ax,'on');

    % Style
    ax.FontSize  = FS_ax;
    ax.LineWidth = LW_ax;
    grid(ax,'on'); box(ax,'on');
    if useGrayAxes, ax.Color = [0.98 0.98 0.98]; end

    for n = 1:N
        if isempty(pathInfo_agent_chain{n}) || numel(pathInfo_agent_chain{n}) < kn, continue; end
        chain = pathInfo_agent_chain{n}{kn};

        posi = find(chain == a, 1, 'first');
        if isempty(posi), continue; end
        pos_int = ceil(posi/2);

        st = x_prev{a}{n}(kn);
        if isnan(st), continue; end

        if a <= 8
            en = y_prev{a}{n}(kn);
        else
            en = st + 0.05;  % terminal short bar, but visible
        end
        if isnan(en) || en <= st, continue; end

        draw_bar(ax, st, en, n, N, colors, pos_int, barH, FS_txt, LW_bar);
    end

    % xlim(ax,[0 x_max]);
    xlim(ax,[t_min t_max]);
    ylim(ax,[0.5, N+1]);
    yticks(ax,1:N);
    yticklabels(ax, flip(arrayfun(@(nn) ['V', num2str(nn)], 1:N, 'UniformOutput', false)));

    title(ax, ['Agent: ', agent_names2{idx}], 'FontSize',FS_tit);

    if idx ~= 5
        ax.XTickLabel = [];
    else
        xlabel(ax,'Time','FontSize',FS_ax);
    end
end
sgtitle('Road Agents (5–8) and Terminal Agent (9) Gantt Chart','FontSize',FS_sgt);

tight_subplots(gcf, 5, 1, gap, marg_h, marg_w);

end

% =================== helper functions ===================
function [t_min, t_max] = compute_xlim(x_prev, y_prev, pathInfo_agent_chain, a_start, a_end, N, kn)
% Compute x-axis limits based on earliest start and latest end among
% tasks that actually exist AND are on the chain (i.e., will be plotted).

t_min = +inf;
t_max = -inf;

for a = a_start:a_end
    for n = 1:N
        % must be on chain, otherwise it won't be plotted
        if isempty(pathInfo_agent_chain{n}) || numel(pathInfo_agent_chain{n}) < kn
            continue;
        end
        chain = pathInfo_agent_chain{n}{kn};
        if isempty(find(chain == a, 1, 'first'))
            continue;
        end

        % start
        if a <= numel(x_prev) && ~isempty(x_prev{a}) && ~isempty(x_prev{a}{n}) && numel(x_prev{a}{n}) >= kn
            st = x_prev{a}{n}(kn);
        else
            continue;
        end
        if isnan(st), continue; end

        % end
        if a <= 8
            if a <= numel(y_prev) && ~isempty(y_prev{a}) && ~isempty(y_prev{a}{n}) && numel(y_prev{a}{n}) >= kn
                en = y_prev{a}{n}(kn);
            else
                continue;
            end
        else
            % terminal agent 9: fabricate a tiny bar
            en = st + 0.05;
        end
        if isnan(en) || en <= st
            continue;
        end

        t_min = min(t_min, st);
        t_max = max(t_max, en);
    end
end

% Fallback if nothing valid
if ~isfinite(t_min) || ~isfinite(t_max)
    t_min = 0;
    t_max = 10;
end

% Add small margins
pad = max(0.5, 0.05*(t_max - t_min));  % 5% or at least 0.5s
t_min = max(0, t_min - pad);           % keep >=0 if you want
t_max = t_max + pad;
end

% function x_max = compute_xmax(x_prev, y_prev, a_start, a_end, N)
% x_max = 0;
% for a = a_start:a_end
%     for n = 1:N
%         if a <= numel(x_prev) && ~isempty(x_prev{a}) && ~isempty(x_prev{a}{n})
%             x_max = max(x_max, max(x_prev{a}{n}(:)));
%         end
%         if a <= 8 && a <= numel(y_prev) && ~isempty(y_prev{a}) && ~isempty(y_prev{a}{n})
%             x_max = max(x_max, max(y_prev{a}{n}(:)));
%         end
%     end
% end
% x_max = x_max + 5; % smaller margin than +10
% end
%% -----------------draw bar function-------------------------------------
function draw_bar(ax, st, en, n, N, colors, pos_int, barH, FS_txt, LW_bar)
dur  = en - st;
yRow = N - n + 1;   % baseline (灰线也应该画在 yRow)

% 条底边 = baseline
rectangle(ax, 'Position', [st, yRow, dur, barH], ...
    'FaceColor', colors(n,:), 'EdgeColor', [0.53 0.81 0.92], 'LineWidth', LW_bar);

% 文字放在条中间
text(ax, st + dur/2, yRow + barH/2, sprintf('V%d-K%d', n, pos_int), ...
    'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
    'FontSize', FS_txt, 'Color','k', 'HitTest','off');
end

%% -----------------tight subplots---------------------------------------
function tight_subplots(fig, nRow, nCol, gap, marg_h, marg_w)
% Tighten subplot spacing (works with subplot-generated axes).
% gap: vertical gap between axes (0~0.1)
% marg_h: top/bottom margin
% marg_w: left/right margin

axs = findall(fig,'Type','axes');
axs = flipud(axs); % keep creation order (subplot order)

axH = (1 - 2*marg_h - (nRow-1)*gap) / nRow;
axW = (1 - 2*marg_w - (nCol-1)*gap) / nCol;

k = 0;
for r = 1:nRow
    for c = 1:nCol
        k = k + 1;
        if k > numel(axs), return; end
        left   = marg_w + (c-1)*(axW+gap);
        bottom = 1 - marg_h - r*axH - (r-1)*gap;
        set(axs(k),'Position',[left bottom axW axH]);
    end
end
end
