%Main program for multi-traffic intersection with c-admm
clear all;
%Time_begin = tic;
% ===================== Batch settings =====================
numVehiclesList = [8];     % 你要测试的车辆数量
seedList        = [4127]; % 你要测试的随机种子
paths = renke_project_paths();
rootSaveDir = paths.batchDir;

% ===================== Traffic generation mode =====================
% 'balanced' : 每个路口负载尽量均等（现有方式，保留）
% 'skewed'   : 故意让某一个路口负载极高，其余较轻（新极端case）
trafficMode      = 'skewed';  % ← 改这里切换模式 'balanced'
hotIntersection  = 1;           % 仅 skewed 模式有效：哪个路口成为热点 (1–4)
hotFraction      = 0.65;        % 仅 skewed 模式有效：热点路口的车辆占比目标

if ~exist(rootSaveDir, 'dir')
    mkdir(rootSaveDir);
end
% ===================== Parallel pool (10 workers) =====================
if license('test','Distrib_Computing_Toolbox')
    p = gcp('nocreate');
    if isempty(p)
        parpool('local', 10);   % 你电脑最多10个
    end  
end
%----------------------------------------------------------------------
for iN = 1:numel(numVehiclesList)
    for iS = 1:numel(seedList)

        close all;
        Time_begin = tic;

        Nveh = numVehiclesList(iN);
        seed = seedList(iS);

        caseName = sprintf('seed_%d_N_%d', seed, Nveh);
        caseDir  = fullfile(rootSaveDir, caseName);
        if ~exist(caseDir, 'dir')
            mkdir(caseDir);
        end

        fprintf('\n====================================================\n');
        fprintf('Running case: %s\n', caseName);
        fprintf('====================================================\n');
%% -----------------------ADMM penalty parameters-------------------------
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 200; 
tol_r = 5*1e-3; tol_s = 5*1e-3; 
T_val = 4.5;       % 同一entrance 两辆车的时间间隔（与固定demo一致）

%% Local Intersection Information
IntSpaceDB = makeIntSpaceDB(); 
LocalTreeCache = cell(9,1); %保存local 调度结果绘图
%---------------------------------------------------------------------
if strcmp(trafficMode, 'skewed')
    [config, vehicleList, stats] = generateSkewedTrafficConfig(Nveh, ...
        'Seed', seed, ...
        'HotIntersection', hotIntersection, ...
        'HotFraction',     hotFraction, ...
        'MaxPerEntrance',  3);
else
    [config, vehicleList, stats] = generateBalancedTrafficConfig(Nveh, ...
        'Seed', seed, ...
        'MaxPerEntrance', 3, ...
        'EntrancePenalty', 0.4);
end
configFile = fullfile(caseDir, sprintf('config_seed%d_N%d.m', seed, Nveh));
printTrafficConfig(config, configFile);
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

N = length(config);
pathInfo = getVehiclePaths(config);

% NOTE: pathInfo_c derived from IntSpaceDB.routeDur — no separate cTime needed.
pathInfo_agent_chain = cell(1,N);
pathInfo_c           = cell(1,N);
for n = 1:N
    kn = 1;
    pathInfo_agent_chain{n} = cell(1, kn);
    pathInfo_c{n}           = cell(1, 1);
    int_seq  = pathInfo{n}(kn).int;
    routeIds = pathInfo{n}(kn).routeId;   % per-intersection route ID

    % Derive traversal time for each intersection from IntSpaceDB
    dur = zeros(1, numel(int_seq));
    for k_int = 1:numel(int_seq)
        ag  = int_seq(k_int);
        rId = routeIds(k_int);
        dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});
    end

    ag_chain = [];
    for i = 1:length(int_seq)-1
        inter1     = int_seq(i);
        inter2     = int_seq(i+1);
        road_agent = getRoadAgent(inter1, inter2);
        ag_chain   = [ag_chain, inter1, road_agent];
    end
    ag_chain = [ag_chain, int_seq(end), 9];
    pathInfo_agent_chain{n}{kn} = ag_chain;
    pathInfo_c{n}{kn}           = dur;
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

Dt = 2; v_max = (100/Dt)*ones(1,N); % Dt=3s，与固定demo一致；v_max由Dt推导保持一致
%% --------------------task periods-------------------------------------------
T = cell(N, 1);
T_final = 16.111;

for n = 1:N
    T{n} = {T_final};
end
%% ----------------initialize vehicles' position and speed-------------------
d1 = zeros(1,N);
detect_range = 510 * ones(1,N);
W = 20;

headway = T_val;   % same-entrance exact spacing
% randWindow = 5.0;  % 每个入口第一辆车的随机偏移范围：0~3 s
% [alpha_tilde, initial_position, init_p_vehi_1, entranceOffset] = ...
%     generateAlphaTildeByEntrance(config, detect_range, v_max, d1, W, headway, randWindow, seed);
minGap = 0.0;
maxGap = 0.0;
entranceOffset = generateSeparatedEntranceOffsets(8, minGap, maxGap, seed);

[alpha_tilde, initial_position, init_p_vehi_1] = ...
    generateAlphaTildeByEntranceFixedOffsets(config, detect_range, v_max, d1, W, headway, entranceOffset);

fprintf('Entrance offsets (s) = %s\n', mat2str(entranceOffset,3));

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
% const.scheduler_mode = 'CR-MPC';     % 'FCFS' or 'CR-MPC'
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

% ===== timing logs =====
iter_time = zeros(max_iter,1);          % 每次迭代总耗时（sec）
agent_time = NaN(max_iter,9);           % 每轮每个agent耗时（sec）
agent_worker = NaN(max_iter,9);         % 每轮每个agent跑在哪个worker上（task.ID）
worker_time = zeros(max_iter, 10);      % 每轮每个worker累计耗时（sec） 你最多10个pool
%%  -------------------主循环-------------------------------------------
N = const.N; max_iter = const.max_iter; tol_r = const.tol_r; tol_s = const.tol_s; rho1 = const.rho1;

for k = 1 : max_iter
    k
    t_iter = tic;
    x_last = x_prev; y_last = y_prev; r_local = 0;
%% Step 1: update dual variables (PARFOR over n)
vehUpd = cell(N,1);

for n = 1:N
    kn = 1;
    chain = const.pathInfo_agent_chain{n}{kn};

    % 本地容器：避免 parfor 直接写 a_x_new{..}{n} 这种嵌套 cell
    ax_loc   = cell(1,9);
    ay_loc   = cell(1,9);
    xbar_loc = cell(1,9);
    ybar_loc = cell(1,8);

    for ag = 1:9
        ax_loc{ag}   = a_x{ag}{n};
        ay_loc{ag}   = a_y{ag}{n};
        xbar_loc{ag} = x_prev_bar{ag}{n};
        if ag <= 8
            ybar_loc{ag} = y_prev_bar{ag}{n};
        end
    end

    % 第一个 agent：只初始化 x_bar，不更新 dual
    ag0 = chain(1);
    xbar_loc{ag0}(kn) = (x_prev{ag0}{n}(kn) + 0) / 2;

    for pos = 2:length(chain)
        prev_ag = chain(pos - 1);
        curr_ag = chain(pos);

        % curr_ag=9 时 prev_ag 一定 <=8，所以 y_prev{prev_ag} 安全
        ax_loc{curr_ag}(kn) = a_x{curr_ag}{n}(kn) + ...
            rho1 * (x_prev{curr_ag}{n}(kn) - y_prev{prev_ag}{n}(kn));

        xbar_loc{curr_ag}(kn) = ...
            (x_prev{curr_ag}{n}(kn) + y_prev{prev_ag}{n}(kn)) / 2;

        ay_loc{prev_ag}(kn) = a_y{prev_ag}{n}(kn) + ...
            rho1 * (y_prev{prev_ag}{n}(kn) - x_prev{curr_ag}{n}(kn));

        ybar_loc{prev_ag}(kn) = ...
            (y_prev{prev_ag}{n}(kn) + x_prev{curr_ag}{n}(kn)) / 2;
    end

    vehUpd{n} = struct('ax',{ax_loc}, 'ay',{ay_loc}, 'xbar',{xbar_loc}, 'ybar',{ybar_loc});
end

% ---- 串行回写 ----
for n = 1:N
    ax_loc   = vehUpd{n}.ax;
    ay_loc   = vehUpd{n}.ay;
    xbar_loc = vehUpd{n}.xbar;
    ybar_loc = vehUpd{n}.ybar;

    for ag = 1:9
        a_x_new{ag}{n}    = ax_loc{ag};
        a_y_new{ag}{n}    = ay_loc{ag};
        x_prev_bar{ag}{n} = xbar_loc{ag};
        if ag <= 8
            y_prev_bar{ag}{n} = ybar_loc{ag};
        end
    end
end

%% Step 2: parallel local updates (Barrier)
f = parallel.FevalFuture.empty(0,9);
meta = cell(1,9);   % 每个 agent 返回一个 struct

% -------- 提交 9 个 agent 的任务 --------
for agent_i = 1:9
    %fprintf('[Iter %d] submitting agent %d...\n', k, agent_i);
    if agent_i >= 1 && agent_i <= 4
        entries = agent_participation{agent_i};
        valid_systems = find(~cellfun(@isempty, entries))';

        if isempty(valid_systems)
            f(agent_i) = parfeval(@agent_update_intersection_stub_single, 1, agent_i);
        else
            f(agent_i) = parfeval(@agent_update_intersection_single, 1, ...
                const, agent_i, entries, valid_systems, ...
                x_prev, y_prev, ...
                x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                a_x_new{agent_i}, a_y_new{agent_i}, ...
                LocalTreeCache{agent_i}, k);
        end

    elseif agent_i >= 5 && agent_i <= 8
        entries = agent_participation{agent_i};
        valid_systems = find(~cellfun(@isempty, entries))';

        if isempty(valid_systems)
            f(agent_i) = parfeval(@agent_update_road_stub_single, 1, agent_i);
        else
            f(agent_i) = parfeval(@agent_update_road_single, 1, ...
                const, agent_i, entries, valid_systems, ...
                x_prev{agent_i}, y_prev{agent_i}, ...
                x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                a_x_new{agent_i}, a_y_new{agent_i});
        end

    else % agent 9
        f(agent_i) = parfeval(@agent_update_terminal_single, 1, ...
            const, x_prev{9}, x_prev_bar{9}, a_x_new{9});
    end
    %fprintf('[Iter %d] submitted agent %d\n', k, agent_i);
end

% % -------- Barrier：等待最慢的也完成 --------
% wait(f);
% 
% % -------- 收集结果（哪个失败就报哪个）--------
% for agent_i = 1:9
%     try
%         meta{agent_i} = fetchOutputs(f(agent_i));
%     catch ME
%         fprintf(2,'\n[Iter %d] Agent %d error report:\n%s\n', k, agent_i, getReport(ME,'extended','hyperlinks','off'));
%         rethrow(ME);
%     end
% end
% -------- 不再用 wait(f)，改成边完成边收集 --------
meta = cell(1,9);
done = false(1,9); 

for ii = 1:9
    %fprintf('[Iter %d] waiting for next completed future...\n', k);
    try
        [completedIdx, S] = fetchNext(f);
        meta{completedIdx} = S;
        done(completedIdx) = true;

        % fprintf('[Iter %d] agent %d finished | kind=%s | time=%.3f | worker=%d\n', ...
        %     k, completedIdx, S.kind, S.elapsed, S.worker);

    catch ME
        fprintf(2, '\n[Iter %d] fetchNext error report:\n%s\n', ...
            k, getReport(ME,'extended','hyperlinks','off'));
        rethrow(ME);
    end
end

%fprintf('[Iter %d] done mask = %s\n', k, mat2str(done));

% for agent_i = 1:9
%     fprintf('[Iter %d] future state of agent %d = %s\n', ...
%         k, agent_i, f(agent_i).State);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -------- 主线程合并写回（只在这里写 x_prev/y_prev/cache）--------
r_local = 0;

for agent_i = 1:9

    S = meta{agent_i};
    fprintf('Agent %d time=%.3f worker=%d\n', agent_i, S.elapsed, S.worker);
    switch S.kind
        case 'intersection'
            % 写回 cache（只有 1..4 有意义）
            LocalTreeCache{agent_i} = S.cache;

            % 写回 x/y（单车系统 kn=1）
            for n = S.valid_systems
                kn = 1;
                x_prev{agent_i}{n}(kn) = S.best_x(n);
                y_prev{agent_i}{n}(kn) = S.best_y(n);

                a_new = S.best_alpha{n}(kn);
                g_new = S.best_gamma{n}(kn);
                r_local = r_local + (S.best_x(n) - a_new)^2 + (S.best_y(n) - g_new)^2;
            end

        case 'road'
            for n = S.valid_systems
                kn = 1;
                x_prev{agent_i}{n}(kn) = S.x_road(n);
                y_prev{agent_i}{n}(kn) = S.y_road(n);
            end

        case 'terminal'
            delay_costs(k) = S.delay_cost;
            for n = 1:N
                x_prev{9}{n}(1) = S.x9_new(n);
            end

        otherwise
            error('Unknown meta.kind = %s', S.kind);
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

r
s
residual_r(k) = r; residual_s(k) = s;

for agent_i = 1:9
    for n = 1:N
        x_hist{agent_i}{n}(1,k) = x_prev{agent_i}{n}(1);
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
iter_time(k) = toc(t_iter);
fprintf('[Iter %d] total time = %.3f s\n', k, iter_time(k));
end

%----------------------------------------------------------------

T_ADMM_TOTAL = toc(Time_begin)/60;   % 结束计时，返回秒数
fprintf('Time elapsed %.3f mins\n', T_ADMM_TOTAL);
% ===== save shared case input for FCFS / other methods =====
caseConfigFile = fullfile(caseDir, 'case_config.mat');
save(caseConfigFile, ...
    'config', 'seed', 'Nveh', 'T_val', ...
    'detect_range', 'v_max', ...
    'alpha_tilde', 'initial_position', ...
    'pathInfo', 'pathInfo_agent_chain', 'pathInfo_c');
% ===== save distributed result =====
matFile = fullfile(caseDir, sprintf('FourIntersection_ADMM_seed%d_N%d.mat',...
    seed, Nveh));
save(matFile, ...
    'config', 'vehicleList', 'stats', ...
    'const', ...
    'residual_r', 'residual_s', ...
    'x_hist', ...
    'x_prev', 'y_prev', ...
    'max_iter', 'k', 'delay_costs', 'LocalTreeCache', ...
    'T_ADMM_TOTAL', 'seed', 'Nveh');

load(matFile);

% plot_ADMM_results(residual_r, residual_s, delay_costs, ...
%     x_prev, y_prev, pathInfo_agent_chain, N, k);
plot_C_ADMM2(residual_r, residual_s, delay_costs, ...
    x_prev, y_prev, const.pathInfo_agent_chain, N, k);

drawnow;  % 确保图已经生成
fig_macro = gcf;
macroFigFile = fullfile(caseDir, sprintf('macro_schedule_seed%d_N%d.png', seed, Nveh));
exportgraphics(gcf, macroFigFile, 'Resolution', 300);
% plot_C_ADMM2 for macro schedule fig

%% ----------Four local scheduling plots-------------------------
% for agent_i = 1:4
%     cache = LocalTreeCache{agent_i};
%     if isempty(cache), continue; end
% 
%     % plot_local_tree_schedule( ...
%     %     cache.NODES, ...
%     %     cache.Path, ...
%     %     cache.Cmat, ...
%     %     agent_i, ...
%     %     cache.valid_systems, ...
%     %     const.pathInfo, cache.pos_map, ...
%     %     't1', [], 'tf', [], ...
%     %     'showReset', true, ...
%     %     'routeLabelMode', 'routeId');
%     fig = plot_local_tree_schedule_compact(cache.NODES, cache.Path, cache.Cmat, agent_i, ...
%     cache.valid_systems, const.pathInfo, cache.pos_map, ...
%     'gap', 0.008, ...                  % 子图更紧
%     'figColor', [0.97 0.97 0.97], ...   % 整张图背景淡灰
%     'axColor',  [0.98 0.98 0.98],...
%     'marg_h', 0.045, ...
%     'title_pad', 0.07);      % 每个子图底色淡灰
% end
%% ---------------------paper use figure-------------------------------------
% for agent_i = 1:4
%     cache = LocalTreeCache{agent_i};
%     if isempty(cache), continue; end
% 
%     fig = plot_local_schedule_final_from_const( ...
%         cache.NODES, ...
%         cache.Path, ...
%         agent_i, ...
%         cache.valid_systems, ...
%         const, ...
%         'figSize', [600 550], ...
%         'gap', 0.004, ...
%         'figColor', [0.97 0.97 0.97], ...
%         'axColor',  [0.98 0.98 0.98], ...
%         'marg_h', 0.07, ...
%         'marg_w', 0.12, ...
%         'title_pad', 0.02);
% end
fig = figure('Color', 'w', 'Position', [60 40 1500 950]);

panelPos = {
    [0.04 0.53 0.44 0.42], ... % Intersection 1
    [0.52 0.53 0.44 0.42], ... % Intersection 2
    [0.04 0.05 0.44 0.42], ... % Intersection 3
    [0.52 0.05 0.44 0.42]  ... % Intersection 4
};

for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    if isempty(cache), continue; end

    p = uipanel('Parent', fig, ...
        'Units', 'normalized', ...
        'Position', panelPos{agent_i}, ...
        'BackgroundColor', [0.97 0.97 0.97], ...
        'BorderType', 'none');

    plot_local_schedule_final_into_panel_1( ...
        p, ...
        cache.NODES, ...
        cache.Path, ...
        agent_i, ...
        cache.valid_systems, ...
        const,...
        'x_prev', x_prev,...
        'gap', 0.004, ...
        'panelColor', 'w', ...
        'axColor', [0.98 0.98 0.98], ...
        'marg_w', 0.12, ...
        'marg_h', 0.08, ...
        'title_pad', 0.06);
end
%localFigFile = fullfile(caseDir, sprintf('local_schedules_seed%d_N%d.png', seed, Nveh));
%exportgraphics(fig, localFigFile, 'Resolution', 300);
savefig(fig, fullfile(caseDir, sprintf('local_schedules_seed%d_N%d.fig', seed, Nveh)));

%% ══════════════════════════════════════════════════════════════════════════
%%  FCFS (Centralized Branch-and-Bound) — same const parameters as ADMM
%% ══════════════════════════════════════════════════════════════════════════
fprintf('\n===== Running FCFS with same config/timing =====\n');

Veh_fcfs = struct([]);
for n = 1:N
    Veh_fcfs(n).entrance   = const.config{n}.entrance;
    Veh_fcfs(n).exit       = const.config{n}.exits;
    Veh_fcfs(n).entryIndex = const.config{n}.entryIndex;
    Veh_fcfs(n).intSeq     = const.pathInfo{n}(1).int;
    Veh_fcfs(n).subDir     = const.pathInfo{n}(1).subDir;
    Veh_fcfs(n).routeId    = const.pathInfo{n}(1).routeId;
    Veh_fcfs(n).NI         = numel(const.pathInfo{n}(1).int);
    Veh_fcfs(n).alpha0     = const.alpha_tilde{n}(1);
end

NI_fcfs = arrayfun(@(v) v.NI, Veh_fcfs);

const_fcfs            = struct();
const_fcfs.N          = N;
const_fcfs.Dt         = const.Dt;
const_fcfs.Veh        = Veh_fcfs;
const_fcfs.IntSpaceDB = const.IntSpaceDB;
const_fcfs.alpha_tilde = const.alpha_tilde;
const_fcfs.Smax       = 3;
const_fcfs.numInt     = 4;
const_fcfs.spacePerInt = 5;
const_fcfs.Mtot       = 4 * 5;
const_fcfs.BIG_M      = 1000;
const_fcfs.NI         = NI_fcfs;

% Root node
Smax_f = const_fcfs.Smax;  Mtot_f = const_fcfs.Mtot;
d0_f   = inf(1,N);  r0_f = zeros(Smax_f,N);  o0_f = zeros(1,N);  ni0_f = zeros(1,N);
U_c0_f = zeros(N,Mtot_f);  U0_f = zeros(N,Mtot_f);
gamma0_f = cell(1,N);  alpha0_node_f = cell(1,N);
x0_f = cell(N,1);  speed0_f = cell(1,N);  ra_reset0_f = zeros(Smax_f,N);

for n = 1:N
    d0_f(n)          = Veh_fcfs(n).alpha0;
    gamma0_f{n}      = NaN(1, NI_fcfs(n));
    alpha0_node_f{n} = NaN(1, NI_fcfs(n));
    x0_f{n}          = cell(1, NI_fcfs(n));
    speed0_f{n}      = 0;
end

NODES_fcfs = {{1,d0_f,r0_f,o0_f,0,ni0_f,0,U_c0_f,U0_f,0, ...
               gamma0_f,0,speed0_f,ra_reset0_f,x0_f,alpha0_node_f}};
OPEN_fcfs  = 1;  LEAF_fcfs = [];  step_fcfs = 0;

Time_fcfs = tic;
while ~isempty(OPEN_fcfs)
    step_fcfs = step_fcfs + 1;
    if step_fcfs > 50000
        error('FCFS: Exceeded max_expand_steps.');
    end
    [~, minIdx_fcfs] = f_min2(NODES_fcfs, OPEN_fcfs);
    [NODES_fcfs, OPEN_fcfs, LEAF_fcfs] = expand_array_global2( ...
        NODES_fcfs, OPEN_fcfs, minIdx_fcfs, LEAF_fcfs, const_fcfs);
end
T_FCFS = toc(Time_fcfs);
fprintf('FCFS elapsed %.2f s  (%d steps)\n', T_FCFS, step_fcfs);

[Path_fcfs, Cost_fcfs, ~] = extract_optimal_path_from_nodes( ...
    NODES_fcfs, LEAF_fcfs, 'criterion', 'g_min');
DATA_fcfs = build_centralized_schedule_data(NODES_fcfs, Path_fcfs, const_fcfs);

fcfsMatFile = fullfile(caseDir, sprintf('FCFS_%s.mat', caseName));
save(fcfsMatFile, 'const_fcfs', 'DATA_fcfs', 'seed', 'Nveh', 'Cost_fcfs');
fprintf('FCFS saved: %s  (cost=%.4f)\n', fcfsMatFile, Cost_fcfs);

fig_fcfs = plot_local_schedule_panel_2x2_centralized(DATA_fcfs, const_fcfs);
drawnow;
exportgraphics(fig_fcfs, fullfile(caseDir, sprintf('fcfs_schedule_%s.png', caseName)), 'Resolution', 300);

fprintf('Finished case: %s\n', caseName);
fprintf('Saved to folder: %s\n', caseDir);
    end
end
%% =========================平行计算用====================================
function S = agent_update_intersection_single(const, agent_i, entries, valid_systems, ...
        x_prev_all, y_prev_all, xi_prev_bar, yi_prev_bar, ai_x, ai_y, cache_ai, k)
    % ✅ timing start + worker id
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    % ✅ make sure cache_ai is a struct
    if isempty(cache_ai) || ~isstruct(cache_ai)
        cache_ai = struct();
    end

    % 直接走你现在的 const 传递版本（不需要 global）
    [best_x, best_y, best_alpha, best_gamma, best_idx, updated_NODES, cache_ai] = ...
        INi_Admm_DecisionTree( ...
            agent_i, entries, ...
            x_prev_all, y_prev_all, ...
            xi_prev_bar, yi_prev_bar, ...
            valid_systems, ai_x, ai_y, const, cache_ai);

    if isempty(cache_ai) || ~isstruct(cache_ai)
    cache_ai = struct();
    end
    cache_ai.iter = k;
     % ✅ timing end
    elapsed = toc(t0);

    S = struct();
    S.kind          = 'intersection';
    S.agent         = agent_i;
    S.valid_systems = valid_systems;
    S.best_x        = best_x;
    S.best_y        = best_y;
    S.best_alpha    = best_alpha;
    S.best_gamma    = best_gamma;
    S.best_idx      = best_idx;
    S.cache         = cache_ai;
    S.elapsed       = elapsed;
    S.worker        = wid;
end

function S = agent_update_road_single(const, agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, xi_prev_bar, yi_prev_bar, ai_x, ai_y)
    % ✅ timing start + worker id
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    [x_road, y_road] = updateRoadAgent( ...
        agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, ...
        xi_prev_bar, yi_prev_bar, ...
        ai_x, ai_y, const);

    % ✅ timing end
    elapsed = toc(t0);

    % 你的单车系统：期望 x_road/y_road 是 N×1（或至少可用 x_road(n)）
    S = struct('kind','road', ...
        'agent',agent_i, ...
        'valid_systems', valid_systems, ...
        'x_road', x_road, ...
        'y_road', y_road,...
        'elapsed', elapsed, ...   % ✅
        'worker', wid);           % ✅
end

function S = agent_update_terminal_single(const, x_prev9, x_prev_bar9, a_x_new9)
    % ✅ timing start + worker id
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    [x9_new, delay_cost] = updateAgent9(x_prev9, x_prev_bar9, a_x_new9, const);

    % ✅ timing end
    elapsed = toc(t0);

    S = struct('kind','terminal', ...
        'x9_new', x9_new, ...
        'delay_cost', delay_cost, ...
        'elapsed', elapsed, ...   % ✅
        'worker', wid);    
end

function S = agent_update_intersection_stub_single(agent_i)
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    S = struct();
    S.kind          = 'intersection';
    S.agent         = agent_i;
    S.valid_systems = [];
    S.best_x        = [];
    S.best_y        = [];
    S.best_alpha    = {};
    S.best_gamma    = {};
    S.best_idx      = [];
    S.cache         = [];
    S.elapsed       = 0;
    S.worker        = wid;
end

function S = agent_update_road_stub_single(agent_i)
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    S = struct('kind','road', 'agent',agent_i, ...
        'valid_systems', [], 'x_road', [], 'y_road', [], ...
        'elapsed', 0, 'worker', wid);
end
