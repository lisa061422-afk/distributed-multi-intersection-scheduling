function plot_ADMM_results(residual_r, residual_s, delay_costs, x_prev, y_prev, pathInfo_agent_chain, N, k)
% plot_ADMM_results
% Plot residuals, delay cost, and machine-centric Gantt charts for agents 1-9.
%
% Inputs:
%   residual_r, residual_s : vectors (length >= k)
%   delay_costs            : vector (length >= k)
%   x_prev                 : 1x9 cell, each x_prev{a}{n}(kn) is scalar
%   y_prev                 : 1x8 cell, each y_prev{a}{n}(kn) is scalar
%   pathInfo_agent_chain   : 1xN cell, each pathInfo_agent_chain{n}{kn} is chain vector
%   N                      : number of systems/vehicles
%   k                      : last iteration index used for plotting

if nargin < 8 || isempty(k)
    k = min([numel(residual_r), numel(residual_s), numel(delay_costs)]);
end

%% Figure 1: Residuals
figure;
semilogy(residual_r(1:k), 'b-', 'DisplayName', 'Primal Residual', 'LineWidth', 2); hold on;
semilogy(residual_s(1:k), 'r--', 'DisplayName', 'Dual Residual', 'LineWidth', 2);
xlabel('Iteration','FontSize',14); ylabel('Residual (log scale)','FontSize', 16);
title('ADMM Residuals over Iterations','FontSize',16);
legend('show'); grid on;

%% Figure 2: Total delay cost
figure;
plot(1:k, delay_costs(1:k), 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
xlabel('Iteration');
ylabel('Total Time Delay Cost');
title('Delay Cost over ADMM Iterations');
grid on;

%% Figure 3: Intersection Agents (1-4) Gantt
figure;
colors = lines(N);
agent_names = {'IN1', 'IN2', 'IN3', 'IN4'};

kn = 1;
x_max = compute_xmax(x_prev, y_prev, 1, 4, N);
for a = 1:4
    subplot(4,1,a); hold on;
    for n = 1:N
       chain = pathInfo_agent_chain{n}{kn};
       posi  = find(chain == a, 1, 'first');   % agent a 在该车 chain 中的位置
       if isempty(posi) || posi >= length(chain), continue; end
       pos_int = ceil(posi/2);
       st = x_prev{a}{n}(kn);
       en = y_prev{a}{n}(kn);

       if ~isnan(st) && ~isnan(en)
           draw_bar(st, en, n, kn, N, colors,pos_int);
       end
    end
    xlim([0, x_max]); ylim([0, N+1]);
    yticks(1:N);
    yticklabels(flip(arrayfun(@(nn) ['V ', num2str(nn)], 1:N, 'UniformOutput', false)));
    title(['Agent: ', agent_names{a}]); grid on; box on;
end
xlabel('Time');
sgtitle('Intersection Agents (1–4) Gantt Chart');

%% Figure 4: Road Agents (5-8) and Terminal (9) Gantt
figure;
agent_names2 = {'RD1', 'RD2', 'RD3', 'RD4', 'x9'};

x_max = compute_xmax(x_prev, y_prev, 5, 9, N); 
for idx = 1:5
    a = idx + 4;  % 5..9
    subplot(5,1,idx); hold on;

    for n = 1:N
        chain = pathInfo_agent_chain{n}{kn};
        posi = find(chain == a, 1);
        if isempty(posi), continue; end
        pos_int = ceil(posi/2);

        st = x_prev{a}{n}(kn);

        if a <= 8 && posi < length(chain)
           en = y_prev{a}{n}(kn);
        else
           en = st + 0.01;  % short bar for terminal
        end

        if ~isnan(st) && ~isnan(en)
           draw_bar(st, en, n, kn, N, colors, pos_int);
        end
    end

    xlim([0, x_max]); ylim([0, N+1]);
    yticks(1:N);
    yticklabels(flip(arrayfun(@(nn) ['V ', num2str(nn)], 1:N, 'UniformOutput', false)));
    title(['Agent: ', agent_names2{idx}]); grid on; box on;
end
xlabel('Time');
sgtitle('Road Agents (5–8) and Terminal Agent (9) Gantt Chart');

end


% =================== helper functions ===================

function x_max = compute_xmax(x_prev, y_prev, a_start, a_end, N)
x_max = 0;
for a = a_start:a_end
    for n = 1:N
        if a <= numel(x_prev) && ~isempty(x_prev{a}) && ~isempty(x_prev{a}{n})
            x_max = max(x_max, max(x_prev{a}{n}(:)));
        end
        if a <= 8 && a <= numel(y_prev) && ~isempty(y_prev{a}) && ~isempty(y_prev{a}{n})
            x_max = max(x_max, max(y_prev{a}{n}(:)));
        end
    end
end
x_max = x_max + 10;
end

function draw_bar(st, en, n, kn, N, colors, pos_int)
dur = max(en - st, 0);
y0  = N - n - 0.15*(kn-1) + 1;
rectangle('Position', [st, y0, dur, 0.9], 'FaceColor', colors(n,:), 'EdgeColor', 'k');
text(st + dur/2, y0 + 0.45, sprintf('%d-IN', pos_int), ...
    'HorizontalAlignment','center', 'FontSize', 10, 'Color','k');
end
