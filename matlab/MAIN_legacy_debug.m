%Main program for multi-traffic intersection with c-admm
clear all;
paths = renke_project_paths();
if ~exist(paths.batchDir, 'dir'), mkdir(paths.batchDir); end
legacyMatFile = fullfile(paths.batchDir, 'FourIntersection_ADMM_legacy_debug.mat');
Time_begin = tic;
% global N rho1 rho2 alpha_tilde NI initial_position Dt deadline...
%     pathInfo_agent_chain pathInfo_c agent_participation pathInfo weight

%% -----------------------ADMM penalty parameters-------------------------
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 100; 
T_val = 2;       % 前 K-1 个任务的周期
tol_r = 1e-2; tol_s = 1e-2;  

%% Local Intersection Information
IntSpaceDB = makeIntSpaceDB(); 
LocalTreeCache = cell(4,1); %保存local 调度结果绘图
%-------------------------------------------------------------------
% config = {  %simu1
%     struct('entrance', 1, 'exits', [6 7 2]),
%     struct('entrance', 4, 'exits', [1 3]),
% }; 
% config = {  %simu1                      % 02052026
%     struct('entrance', 1, 'exits', [7]),
%     struct('entrance', 4, 'exits', [1 3]),
%     struct('entrance', 6, 'exits', [1]),
% }; 
% config = {  %simu1                      % 02052026
%     struct('entrance', 1, 'exits', [7 3]),
%     struct('entrance', 4, 'exits', [1 3]),
%     struct('entrance', 6, 'exits', [1]),
%     struct('entrance', 7, 'exits', [4]),
%     struct('entrance', 5, 'exits', [3]),
%     struct('entrance', 3, 'exits', [2]),
% }; 
% config = {  %simu1                      % 02052026
%     struct('entrance', 4, 'exits', [5]),
%     struct('entrance', 2, 'exits', [3]),
%     struct('entrance', 1, 'exits', [8]),
%     struct('entrance', 7, 'exits', [6]),
%     struct('entrance', 6, 'exits', [7]),
% }; 
config = {  %simu1                      % 02052026
    struct('entrance', 1, 'exits', [7 3]),
    struct('entrance', 4, 'exits', [1 3]),
    struct('entrance', 6, 'exits', [1]),
    struct('entrance', 7, 'exits', [4]),
    struct('entrance', 5, 'exits', [3]),
    struct('entrance', 3, 'exits', [2 4]),
    struct('entrance', 2, 'exits', [5]),
}; 
% ---- expand route-based config -> vehicle-based config ----
vehicleConfig = {};
vid = 0;
for g = 1:length(config)
    ent = config{g}.entrance;
    exits = config{g}.exits;

    for j = 1:length(exits)
        vid = vid + 1;
        vehicleConfig{vid} = struct( ...
            'entrance', ent, ...
            'exits', exits(j), ...     % 单车
            'entryIndex', j ...        % ✅ 同一entrance下的第j辆
        );
    end
end
config = vehicleConfig;

cTime = [2.3562, 2, 0.7854];  % [左, 直, 右] 的时间成本 后面要取消
N = length(config);
pathInfo = getVehiclePaths(config);

pathInfo_agent_chain = cell(1,N); %pathInfo_c = cell(1,N);
for n = 1:N
    kn = 1;
    pathInfo_agent_chain{n} = cell(1, kn);
    %pathInfo_c{n} = cell(1, 1);
    int_seq = pathInfo{n}(kn).int;
    sub_dir = pathInfo{n}(kn).subDir;
    dur = cTime(sub_dir);  % 每段执行时间
    % 构建 agent_chain: inter1 → road → inter2 → ... → interN → 9
    ag_chain = [];
    for i = 1:length(int_seq)-1
        inter1 = int_seq(i);
        inter2 = int_seq(i+1);
        road_agent = getRoadAgent(inter1, inter2);
        ag_chain = [ag_chain, inter1, road_agent];
    end
    ag_chain = [ag_chain, int_seq(end), 9];  % 最后加上终点 intersection 和 virtual agent
    pathInfo_agent_chain{n}{kn} = ag_chain; %每一辆车路过哪些agents
    pathInfo_c{n}{kn} = dur; %每一辆车在每个路口对应的c
end

%% 保存变量 agent_participation
% agent_participation{ag}{n} 非空 => 车辆/系统 n 经过 agent ag
agent_participation = repmat({cell(N, 1)}, 8, 1);

for n = 1:N
    kn = 1;
    chain = pathInfo_agent_chain{n}{kn};
    ags = chain(1:end-1);   % 不含终点9
    for ii = 1:numel(ags)
        ag = ags(ii);
        agent_participation{ag}{n} = 1;   % 不再 append [.., kn]
    end
end

v_max = 20*ones(1,N); Dt = 150/20; %shorest time between two intersections
%% --------------------task periods-------------------------------------------
T = cell(N, 1);
%NI = ones(1, N);      % 每辆车=一个system => 只有一个task

T_final = 16.111;

for n = 1:N
    T{n} = {T_final};
end
%% ----------------initialize vehicles' position and speed-------------------
d1 = zeros(1,N); % 你也可以给不同入口不同偏移
detect_range = 510 * ones(1,N); 
%% 手动修改最早抵达时间
detect_range(5) = 510 - 2 * 0.3 * v_max(6); 
W = 20;


headway = T_val;   % 用你原来的 T_val 当入口车间隔（也可以自己设 2.0）

init_p_vehi_1 = zeros(N,1);
initial_position = zeros(N,1);   % 单车系统：每车一个位置
alpha_tilde = cell(N,1);

for n = 1:N
    alpha_tilde{n} = zeros(1,1);
    base = (detect_range(n)/2 - W/2) / v_max(n) + d1(n);

    % ✅ 同一 entrance 的第 entryIndex 辆车，延后 (entryIndex-1)*headway
    alpha_tilde{n}(1) = base + (config{n}.entryIndex - 1) * headway;

    % 用同一个到达时间来定义初始位置（保持物理一致）
    t_arr = alpha_tilde{n}(1);
    init_p_vehi_1(n) = -(detect_range(n)/2 - W/2 + t_arr * v_max(n));
    initial_position(n,1) = init_p_vehi_1(n);
end

% I = ones(1,N);   % 每车只有1个task
speed = cell(N,1);
for n = 1:N
    speed{n} = 0;
end
%% Earliest Exit Times
deadline = cell(N, 1);
for n = 1:N
    kn = 1;
    assert(numel(alpha_tilde{n}) == 1, 'Expected alpha_tilde{n} length = 1.');
    deadline{n} = zeros(kn, 1);
    durations = pathInfo_c{n}{kn};
    c_total = sum(durations);

    chain = pathInfo_agent_chain{n}{kn};
    num_roads = floor((length(chain) - 1) / 2);

    deadline{n}(kn) = alpha_tilde{n}(kn) + (c_total + Dt * num_roads);
end

%% ===================== Pack constants into const =====================
const = struct();
const.scheduler_mode = 'FCFS';     % 'FCFS' or 'CR-MPC'
% --- scalar parameters ---
const.rho1   = rho1; const.rho2   = rho2; const.weight = weight; const.max_iter = max_iter;
const.tol_r = tol_r; const.tol_s = tol_s;
% --- problem size ---
const.N  = N;
% --- timing / physical parameters ---
const.Dt = Dt;
const.deadline = deadline;           % cell(N,1)
const.alpha_tilde = alpha_tilde;     % cell(N,1)
const.initial_position = initial_position; % N×1

% --- routing / topology / participation ---
const.config = config;                          % vehicle config
const.pathInfo = pathInfo;                      % from getVehiclePaths
const.pathInfo_agent_chain = pathInfo_agent_chain;
const.pathInfo_c = pathInfo_c;
const.agent_participation = agent_participation;

% --- intersection DB / caches (read-only DB) ---
const.IntSpaceDB = IntSpaceDB;

% 备注：LocalTreeCache 是会随着迭代变化的（每轮更新 iter、NODES/Path 等）
% 所以它不放 const


%% -----------------------ADMM initialization----------------------------- 
% ===== init with OLD structure: x_prev{agent}{n} is scalar =====
alpha_tilde = const.alpha_tilde;
[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N);

% x_prev = cell(1,9);
% y_prev = cell(1,8);
% x_prev_bar = cell(1,9);
% y_prev_bar = cell(1,8);
% 
% % 1) all NaN
% for i = 1:9
%     x_prev{i} = cell(1,N);
%     x_prev_bar{i} = cell(1,N);
%     [x_prev{i}{:}]     = deal(NaN);
%     [x_prev_bar{i}{:}] = deal(NaN);
% end
% for i = 1:8
%     y_prev{i} = cell(1,N);
%     y_prev_bar{i} = cell(1,N);
%     [y_prev{i}{:}]     = deal(NaN);
%     [y_prev_bar{i}{:}] = deal(NaN);
% end
% 
% % 2) fill participating vehicles for agents 1..8
% for agent_i = 1:8
%     entries = agent_participation{agent_i};
%     if all(cellfun(@isempty, entries)), continue; end
%     valid_systems = find(~cellfun(@isempty, entries))';
% 
%     for n = valid_systems
%         x_prev{agent_i}{n}     = alpha_tilde{n};   % kn=1 => scalar
%         x_prev_bar{agent_i}{n} = alpha_tilde{n};
%         y_prev{agent_i}{n}     = alpha_tilde{n};
%         y_prev_bar{agent_i}{n} = alpha_tilde{n};
%     end
% end
% 
% % 3) agent 9: x exists for all vehicles
% for n = 1:N
%     x_prev{9}{n}     = alpha_tilde{n};
%     x_prev_bar{9}{n} = alpha_tilde{n}; 
% end

% 4) duals: zeros (keep old structure)
a_x = cell(1,9); a_y = cell(1,9);
a_x_new = cell(1,9); a_y_new = cell(1,9);
for i = 1:9
    a_x{i} = cell(1,N); a_y{i} = cell(1,N);
    a_x_new{i} = cell(1,N); a_y_new{i} = cell(1,N);
    [a_x{i}{:}]     = deal(0);
    [a_y{i}{:}]     = deal(0);
    [a_x_new{i}{:}] = deal(0);
    [a_y_new{i}{:}] = deal(0);
end


residual_r = zeros(max_iter, 1); residual_s = zeros(max_iter, 1); 
delay_costs = zeros(max_iter, 1);
x_hist = cell(1, 9);  % 每个 agent 维护一个 cell
for i = 1:9
    x_hist{i} = cell(1, N);  % 每个 agent 记录每个系统的轨迹
    for n = 1:N
        K = length(alpha_tilde{n});  % 每个系统的 recurring task 数量
        x_hist{i}{n} = NaN(K, max_iter);  % 每一列是一次迭代的 x(n,k)
    end
end

%%  -------------------主循环-------------------------------------------
N = const.N; max_iter = const.max_iter; tol_r = const.tol_r; tol_s = const.tol_s;

for k = 1 : max_iter
    k
    x_last = x_prev; y_last = y_prev; r_local = 0;
    %% Step 1: update dual variables
    for n = 1:N
        kn = 1;
        chain = const.pathInfo_agent_chain{n}{kn};

        % 第一个 agent：只初始化 x_bar，不更新 dual
        ag0 = chain(1);
        x_prev_bar{ag0}{n}(kn) = (x_prev{ag0}{n}(kn) + 0) / 2;

        for pos = 2:length(chain)
            prev_ag = chain(pos - 1);
            curr_ag = chain(pos);

            % 更新当前 agent 的 dual: x - y
            a_x_new{curr_ag}{n}(kn) = a_x{curr_ag}{n}(kn) + ...
                 rho1 * (x_prev{curr_ag}{n}(kn) - y_prev{prev_ag}{n}(kn));

            x_prev_bar{curr_ag}{n}(kn) = ...
                (x_prev{curr_ag}{n}(kn) + y_prev{prev_ag}{n}(kn)) / 2;

            % 更新前一个 agent 的 dual: y - x
            a_y_new{prev_ag}{n}(kn) = a_y{prev_ag}{n}(kn) + ...
                rho1 * (y_prev{prev_ag}{n}(kn) - x_prev{curr_ag}{n}(kn));

            y_prev_bar{prev_ag}{n}(kn) = ...
                (y_prev{prev_ag}{n}(kn) + x_prev{curr_ag}{n}(kn)) / 2;
        end
    end
%% Step 2: update primal variables
% 假设有：
% - pathInfo{n}：包含每条路径的 agent_chain
% - agent_participation{i}：agent i 参与的 routes（如之前建立）
% - x_prev, y_prev, a{i}：每个 agent 的前一轮状态和 dual

for agent_i = 1:9
    if agent_i >= 1 && agent_i <= 4
    % ========== Intersection Agent: 调用局部优化器 ==========
    entries = agent_participation{agent_i};  % cell(N,1)，每个 cell 非空表示车辆n经过该agent
    if all(cellfun(@isempty, entries))
        continue;
    end
    valid_systems = find(~cellfun(@isempty, entries))';

    % 调用局部优化器
    [best_x, best_y, best_alpha, best_gamma, best_idx, updated_NODES,LocalCache] = ...
        INi_Admm_DecisionTree( ...
            agent_i, entries, ...
            x_prev, y_prev, ...
            x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
            valid_systems, a_x_new{agent_i}, a_y_new{agent_i}, const,LocalTreeCache);
    LocalTreeCache{agent_i} = LocalCache;
    LocalTreeCache{agent_i}.iter = k;

    % ========== 更新当前 agent 涉及的所有车辆系统 ==========
    for n = valid_systems
        if isempty(entries{n}) continue; end
        kn = 1;  % ✅ 单车系统固定

        x_new = best_x(n);
        y_new = best_y(n);
        x_prev{agent_i}{n}(kn) = x_new;
        y_prev{agent_i}{n}(kn) = y_new;

        a_new = best_alpha{n}(kn);
        g_new = best_gamma{n}(kn);

        r_local = r_local + (x_new - a_new)^2;
        r_local = r_local + (y_new - g_new)^2;
    end
    elseif agent_i >= 5 && agent_i <= 8
    % ========== Road Agent：直接求解 min Frobenius norm + 拉格朗日项 ==========
    entries = agent_participation{agent_i};                 % cell(N,1)，非空表示车辆n经过该agent
    if all(cellfun(@isempty, entries))
        continue;
    end
    valid_systems = find(~cellfun(@isempty, entries))';

    [x_road, y_road] = updateRoadAgent( ...
        agent_i, entries, valid_systems, ...
        x_prev{agent_i}, y_prev{agent_i}, ...
        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
        a_x_new{agent_i}, a_y_new{agent_i},const);

    % 更新：每车只有 kn=1
    for n = valid_systems
        if isempty(entries{n})
            continue;
        end
        kn = 1;
        % 兼容 updateRoadAgent 输出是 N×K 或 N×1
        if ismatrix(x_road) && size(x_road,2) >= kn
            x_prev{agent_i}{n}(kn) = x_road(n,kn);
            y_prev{agent_i}{n}(kn) = y_road(n,kn);
        else
            x_prev{agent_i}{n}(kn) = x_road(n);
            y_prev{agent_i}{n}(kn) = y_road(n);
        end
    end
    elseif agent_i == 9
    % ========== Agent 9 (cost function terminal) ==========
    [x9_new, delay_cost] = updateAgent9(x_prev{9}, x_prev_bar{9}, a_x_new{9},const);
    delay_costs(k) = delay_cost;

    % 每车一个task: kn=1
    for n = 1:N
        x_prev{agent_i}{n}(1) = x9_new(n);
    end

    end

end
%% Step 3: residuals
r = compute_r(x_prev, y_prev, r_local,const);
s = 0;
% agents 1..8: 有 x/y，且按 entries 过滤
for agent_i = 1:8
    entries = agent_participation{agent_i};
    if all(cellfun(@isempty, entries)), continue; end
    valid_systems = find(~cellfun(@isempty, entries))';

    for n = valid_systems
        s = s + norm(x_prev{agent_i}{n} - x_last{agent_i}{n})^2 + ...
            norm(y_prev{agent_i}{n} - y_last{agent_i}{n})^2;
    end
end
% agent 9: 只有 x
for n = 1:N
    s = s + norm(x_prev{9}{n} - x_last{9}{n})^2;
end

residual_r(k) = r; residual_s(k) = s;

for agent_i = 1:9
    for n = 1:N
        x_hist{agent_i}{n}(1,k) = x_prev{agent_i}{n}(kn);
    end
end

a_x = a_x_new; a_y = a_y_new;
%% Stopping Criteria
if r < tol_r && s < tol_s
    fprintf('Converged at iteration %d\n', k);
    residual_r = residual_r(1:k);
    residual_s = residual_s(1:k);
    for i = 1:9
        x_hist{i} = cellfun(@(v) v(1:k), x_hist{i}, 'UniformOutput', false);
    end
    break;
end

end
%% ----------Four local scheduling plots-------------------------
for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    if isempty(cache), continue; end

    plot_local_tree_schedule( ...
        cache.NODES, ...
        cache.Path, ...
        cache.Cmat, ...
        agent_i, ...
        cache.valid_systems, ...
        const.pathInfo, cache.pos_map, ...
        't1', [], 'tf', [], ...
        'showReset', true, ...
        'routeLabelMode', 'routeId');
end
%----------------------------------------------------------------

T_ADMM_TOTAL = toc(Time_begin)/60;   % 结束计时，返回秒数
fprintf('Time elapsed %.3f mins\n', T_ADMM_TOTAL);

save(legacyMatFile, ...
    'config', ...
    'const', ...
    'residual_r', 'residual_s', ...
    'x_hist', ...
    'x_prev', 'y_prev', ...
    'max_iter', 'k', 'delay_costs');

load(legacyMatFile);

% plot_ADMM_results(residual_r, residual_s, delay_costs, ...
%     x_prev, y_prev, pathInfo_agent_chain, N, k);
plot_C_ADMM(residual_r, residual_s, delay_costs, ...
    x_prev, y_prev, const.pathInfo_agent_chain, N, k);

