% RUN_MultiStart_Experiment.m
% 5 random vehicle configs (N=15), each run with:
%   1 deterministic init (randInitScale=0)  +  nRandStarts random inits (randInitScale=2)
% Saves BatchRuns/multistart_results.csv, then ready for PPTX generation.

clear all;

% ── Settings ───────────────────────────────────────────────────────────────
seedList      = [306];   % ← 改这一个，跑完换下一个 (301~305)
Nveh_target   = 15;
nRandStarts   = 2;   % ← 每次跑1个随机，手动重复运行
randInitScale = 3.0;
useParallel   = true;

rootSaveDir = 'BatchRuns';
if ~exist(rootSaveDir, 'dir'), mkdir(rootSaveDir); end

% ── Physical / ADMM parameters ─────────────────────────────────────────────
T_val        = 2.0;
T_ent        = 0.0;
Dt           = 2.0;
v_max_phys   = 1.5;
W            = 1.6;
detect_range_val = 7.6;
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 500;
tol_r = 1e-2; tol_s = 1e-2;
demoMaxPerInt = 8;

% ── Parallel pool ──────────────────────────────────────────────────────────
if useParallel && license('test','Distrib_Computing_Toolbox')
    p = gcp('nocreate');
    if isempty(p) || p.NumWorkers ~= 10
        if ~isempty(p), delete(p); end
        parpool('local', 10);
    end
end

IntSpaceDB = makeIntSpaceDB();
ms_rows = {};

% ── Main loop ──────────────────────────────────────────────────────────────
for iS = 1:numel(seedList)
    seed = seedList(iS);
    fprintf('\n====================================================\n');
    fprintf('Case %d/%d  seed=%d  N=%d\n', iS, numel(seedList), seed, Nveh_target);
    fprintf('====================================================\n');

    % Config generation
    [config_raw, ~, ~] = generateBalancedTrafficConfig(Nveh_target, ...
        'Seed', seed, ...
        'MaxPerEntrance',     ceil(Nveh_target/4), ...
        'MaxPerIntersection', demoMaxPerInt, ...
        'MaxPerStage',        ceil(Nveh_target/4)+1, ...
        'EntrancePenalty',    0.4);

    % Expand route-based config → vehicle-based config
    vehicleConfig = {};
    vid = 0;
    for g = 1:length(config_raw)
        ent  = config_raw{g}.entrance;
        exits = config_raw{g}.exits;
        for j = 1:length(exits)
            vid = vid + 1;
            vehicleConfig{vid} = struct('entrance', ent, 'exits', exits(j), 'entryIndex', j);
        end
    end
    config = vehicleConfig;
    N = length(config);

    % Routing
    pathInfo = getVehiclePaths(config);
    pathInfo_agent_chain = cell(1,N);
    pathInfo_c = cell(1,N);
    for n = 1:N
        kn = 1;
        pathInfo_agent_chain{n} = cell(1,1);
        pathInfo_c{n}           = cell(1,1);
        int_seq  = pathInfo{n}(kn).int;
        routeIds = pathInfo{n}(kn).routeId;
        dur = zeros(1, numel(int_seq));
        for k_int = 1:numel(int_seq)
            ag = int_seq(k_int); rId = routeIds(k_int);
            dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});
        end
        ag_chain = [];
        for i = 1:length(int_seq)-1
            ag_chain = [ag_chain, int_seq(i), getRoadAgent(int_seq(i), int_seq(i+1))];
        end
        ag_chain = [ag_chain, int_seq(end), 9];
        pathInfo_agent_chain{n}{kn} = ag_chain;
        pathInfo_c{n}{kn}           = dur;
    end

    % agent_participation
    agent_participation = repmat({cell(N,1)}, 8, 1);
    for n = 1:N
        chain = pathInfo_agent_chain{n}{1};
        ags   = chain(1:end-1);
        for ii = 1:numel(ags)
            agent_participation{ags(ii)}{n} = 1;
        end
    end

    % Physical setup
    v_max        = v_max_phys * ones(1,N);
    detect_range = detect_range_val * ones(1,N);
    all_entrances = cellfun(@(c) c.entrance, config);
    unique_ents   = unique(all_entrances, 'stable');
    ent_rank      = zeros(1, max(unique_ents));
    for ei = 1:numel(unique_ents), ent_rank(unique_ents(ei)) = ei - 1; end

    alpha_tilde      = cell(N,1);
    initial_position = zeros(N,1);
    for n = 1:N
        alpha_tilde{n} = zeros(1,1);
        base = (detect_range(n)/2 - W/2) / v_max(n);
        alpha_tilde{n}(1) = base ...
            + ent_rank(config{n}.entrance) * T_ent ...
            + (config{n}.entryIndex - 1)   * T_val;
        initial_position(n) = -(detect_range(n)/2 - W/2 + alpha_tilde{n}(1) * v_max(n));
    end

    % Deadline
    deadline = cell(N,1);
    for n = 1:N
        kn = 1;
        durations = pathInfo_c{n}{kn};
        chain     = pathInfo_agent_chain{n}{kn};
        num_roads = floor((length(chain)-1)/2);
        deadline{n} = alpha_tilde{n}(kn) + sum(durations) + Dt*num_roads;
    end

    % Pack const
    const = struct();
    const.rho1 = rho1; const.rho2 = rho2; const.weight = weight;
    const.max_iter = max_iter; const.tol_r = tol_r; const.tol_s = tol_s;
    const.N = N; const.Dt = Dt; const.priority_n = 0;
    const.use_pruning = true; const.use_weak_rule = true;
    const.timeout_int_s = 30; const.useTBound = true;
    const.use_quadprog = true;
    const.deadline = deadline; const.alpha_tilde = alpha_tilde;
    const.initial_position = initial_position;
    const.config = config; const.pathInfo = pathInfo;
    const.pathInfo_agent_chain = pathInfo_agent_chain;
    const.pathInfo_c = pathInfo_c;
    const.agent_participation = agent_participation;
    const.IntSpaceDB = IntSpaceDB;
    const.useParallel = useParallel;
    const.T_val = T_val; const.T_ent = T_ent;
    const.v_max_phys = v_max_phys; const.W = W;
    const.detect_range_val = detect_range_val;

    % ── Single run (det or rand controlled by randInitScale) ─────────────
    const.randInitScale = randInitScale;
    label = 'det'; if randInitScale > 0, label = sprintf('rand%.1f', randInitScale); end
    fprintf('  [%s]  seed=%d ...\n', label, seed);
    [~,~,~,~,~, dc, kk, Tt] = run_admm_core(const, agent_participation);
    cost = dc(kk);
    fprintf('  [%s]  cost=%.4f  iter=%d  t=%.1fs\n', label, cost, kk, Tt);
    % ── Append this result immediately ───────────────────────────────────
    csvFile = fullfile(pwd, rootSaveDir, 'multistart_results.csv');
    if ~exist(fullfile(pwd, rootSaveDir), 'dir')
        mkdir(fullfile(pwd, rootSaveDir));
    end
    needHeader = ~exist(csvFile, 'file');
    fid = fopen(csvFile, 'a');
    if fid == -1
        warning('Cannot open CSV: %s — printing result to console only.', csvFile);
        fprintf('  RESULT: seed=%d N=%d cost=%.6f iter=%d t=%.2f label=%s\n', ...
            seed, Nveh_target, cost, kk, Tt, label);
    else
        if needHeader
            fprintf(fid, 'seed,N,cost,iter,time_s,label\n');
        end
        fprintf(fid, '%d,%d,%.6f,%d,%.2f,%s\n', seed, Nveh_target, cost, kk, Tt, label);
        fclose(fid);
        fprintf('  => Appended to: %s\n', csvFile);
    end
end

% ══════════════════════════════════════════════════════════════════════════
% Local functions (copied from MAIN_Parallel_Compute.m)
% ══════════════════════════════════════════════════════════════════════════

function [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, ...
          delay_costs, k, T_ADMM] = run_admm_core(const, agent_participation)

N        = const.N;
Dt       = const.Dt;
max_iter = const.max_iter;
tol_r    = const.tol_r;
tol_s    = const.tol_s;
rho1     = const.rho1;
alpha_tilde          = const.alpha_tilde;
pathInfo_agent_chain = const.pathInfo_agent_chain;
pathInfo_c           = const.pathInfo_c;

LocalTreeCache = cell(9,1);
[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N, const.randInitScale);

a_x = cell(1,9); a_y = cell(1,9);
a_x_new = cell(1,9); a_y_new = cell(1,9);
for i = 1:9
    a_x{i} = cell(1,N);    a_y{i} = cell(1,N);
    a_x_new{i} = cell(1,N); a_y_new{i} = cell(1,N);
    [a_x{i}{:}]     = deal(0);
    [a_y{i}{:}]     = deal(0);
    [a_x_new{i}{:}] = deal(0);
    [a_y_new{i}{:}] = deal(0);
end

residual_r  = zeros(max_iter, 1);
residual_s  = zeros(max_iter, 1);
delay_costs = zeros(max_iter, 1);
T0 = tic;

for k = 1 : max_iter
    t_iter = tic;
    x_last = x_prev; y_last = y_prev;

    vehUpd = cell(N,1);
    for n = 1:N
        kn    = 1;
        chain = const.pathInfo_agent_chain{n}{kn};
        ax_loc   = cell(1,9); ay_loc   = cell(1,9);
        xbar_loc = cell(1,9); ybar_loc = cell(1,8);
        for ag = 1:9
            ax_loc{ag}   = a_x{ag}{n};
            ay_loc{ag}   = a_y{ag}{n};
            xbar_loc{ag} = x_prev_bar{ag}{n};
            if ag <= 8, ybar_loc{ag} = y_prev_bar{ag}{n}; end
        end
        ag0 = chain(1);
        xbar_loc{ag0}(kn) = (x_prev{ag0}{n}(kn) + 0) / 2;
        for pos = 2:length(chain)
            prev_ag = chain(pos-1); curr_ag = chain(pos);
            ax_loc{curr_ag}(kn) = a_x{curr_ag}{n}(kn) + ...
                rho1*(x_prev{curr_ag}{n}(kn) - y_prev{prev_ag}{n}(kn));
            xbar_loc{curr_ag}(kn) = (x_prev{curr_ag}{n}(kn) + y_prev{prev_ag}{n}(kn))/2;
            ay_loc{prev_ag}(kn) = a_y{prev_ag}{n}(kn) + ...
                rho1*(y_prev{prev_ag}{n}(kn) - x_prev{curr_ag}{n}(kn));
            ybar_loc{prev_ag}(kn) = (y_prev{prev_ag}{n}(kn) + x_prev{curr_ag}{n}(kn))/2;
        end
        vehUpd{n} = struct('ax',{ax_loc},'ay',{ay_loc},'xbar',{xbar_loc},'ybar',{ybar_loc});
    end
    for n = 1:N
        for ag = 1:9
            a_x_new{ag}{n}    = vehUpd{n}.ax{ag};
            a_y_new{ag}{n}    = vehUpd{n}.ay{ag};
            x_prev_bar{ag}{n} = vehUpd{n}.xbar{ag};
            if ag <= 8, y_prev_bar{ag}{n} = vehUpd{n}.ybar{ag}; end
        end
    end

    meta = cell(1,9);
    if const.useParallel
        f = parallel.FevalFuture.empty(0,9);
        for agent_i = 1:9
            if agent_i <= 4
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
                if isempty(valid_systems)
                    f(agent_i) = parfeval(@agent_update_intersection_stub_single, 1, agent_i);
                else
                    f(agent_i) = parfeval(@agent_update_intersection_single, 1, ...
                        const, agent_i, entries, valid_systems, ...
                        x_prev, y_prev, x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i}, LocalTreeCache{agent_i}, k);
                end
            elseif agent_i <= 8
                entries       = agent_participation{agent_i};
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
            else
                f(agent_i) = parfeval(@agent_update_terminal_single, 1, ...
                    const, x_prev{9}, x_prev_bar{9}, a_x_new{9});
            end
        end
        for ii = 1:9
            try
                [completedIdx, S_res] = fetchNext(f);
                meta{completedIdx} = S_res;
            catch ME
                fprintf(2, '\n[Iter %d] fetchNext error:\n%s\n', k, ...
                    getReport(ME,'extended','hyperlinks','off'));
                rethrow(ME);
            end
        end
    else
        for agent_i = 1:9
            if agent_i <= 4
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
                if isempty(valid_systems)
                    meta{agent_i} = agent_update_intersection_stub_single(agent_i);
                else
                    meta{agent_i} = agent_update_intersection_single(const, agent_i, entries, valid_systems, ...
                        x_prev, y_prev, x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i}, LocalTreeCache{agent_i}, k);
                end
            elseif agent_i <= 8
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
                if isempty(valid_systems)
                    meta{agent_i} = agent_update_road_stub_single(agent_i);
                else
                    meta{agent_i} = agent_update_road_single(const, agent_i, entries, valid_systems, ...
                        x_prev{agent_i}, y_prev{agent_i}, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i});
                end
            else
                meta{agent_i} = agent_update_terminal_single(const, x_prev{9}, x_prev_bar{9}, a_x_new{9});
            end
        end
    end

    r_local = 0;
    for agent_i = 1:9
        S_res = meta{agent_i};
        switch S_res.kind
            case 'intersection'
                LocalTreeCache{agent_i} = S_res.cache;
                for n = S_res.valid_systems
                    kn = 1;
                    x_prev{agent_i}{n}(kn) = S_res.best_x(n);
                    y_prev{agent_i}{n}(kn) = S_res.best_y(n);
                    if ~isempty(S_res.best_alpha{n}) && ~isempty(S_res.best_gamma{n})
                        r_local = r_local + (S_res.best_x(n) - S_res.best_alpha{n}(kn))^2 ...
                                          + (S_res.best_y(n) - S_res.best_gamma{n}(kn))^2;
                    end
                end
            case 'road'
                for n = S_res.valid_systems
                    kn = 1;
                    x_prev{agent_i}{n}(kn) = S_res.x_road(n);
                    y_prev{agent_i}{n}(kn) = S_res.y_road(n);
                end
            case 'terminal'
                delay_costs(k) = S_res.delay_cost;
                for n = 1:N, x_prev{9}{n}(1) = S_res.x9_new(n); end
            otherwise
                error('Unknown meta.kind = %s', S_res.kind);
        end
    end

    for agent_i = 1:9
        for n = 1:N
            if ~isempty(x_prev{agent_i}{n}) && any(isnan(x_prev{agent_i}{n})) ...
                    && ~isempty(x_last{agent_i}{n}) && ~any(isnan(x_last{agent_i}{n}))
                x_prev{agent_i}{n} = x_last{agent_i}{n};
            end
            if agent_i <= 8 && ~isempty(y_prev{agent_i}{n}) && any(isnan(y_prev{agent_i}{n})) ...
                    && ~isempty(y_last{agent_i}{n}) && ~any(isnan(y_last{agent_i}{n}))
                y_prev{agent_i}{n} = y_last{agent_i}{n};
            end
        end
    end

    r = compute_r(x_prev, y_prev, r_local, const);
    s = 0;
    for agent_i = 1:8
        ent = agent_participation{agent_i};
        if all(cellfun(@isempty, ent)), continue; end
        vs = find(~cellfun(@isempty, ent))';
        for n = vs
            s = s + norm(x_prev{agent_i}{n} - x_last{agent_i}{n})^2 + ...
                    norm(y_prev{agent_i}{n} - y_last{agent_i}{n})^2;
        end
    end
    for n = 1:N, s = s + norm(x_prev{9}{n} - x_last{9}{n})^2; end

    residual_r(k) = r; residual_s(k) = s;
    a_x = a_x_new; a_y = a_y_new;

    if r < tol_r && s < tol_s
        fprintf('Converged at iteration %d\n', k);
        residual_r  = residual_r(1:k);
        residual_s  = residual_s(1:k);
        delay_costs = delay_costs(1:k);
        break;
    end
    fprintf('[Iter %d] r=%.4f  s=%.4f  time=%.2fs\n', k, r, s, toc(t_iter));
end

T_ADMM = toc(T0);
fprintf('ADMM elapsed %.1f s  (priority_n=%d)\n', T_ADMM, const.priority_n);
end

function S = agent_update_intersection_single(const, agent_i, entries, valid_systems, ...
        x_prev_all, y_prev_all, xi_prev_bar, yi_prev_bar, ai_x, ai_y, cache_ai, k)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    if isempty(cache_ai) || ~isstruct(cache_ai), cache_ai = struct(); end
    [best_x, best_y, best_alpha, best_gamma, best_idx, ~, cache_ai] = ...
        INi_Admm_DecisionTree(agent_i, entries, x_prev_all, y_prev_all, ...
            xi_prev_bar, yi_prev_bar, valid_systems, ai_x, ai_y, const, cache_ai);
    if isempty(cache_ai) || ~isstruct(cache_ai), cache_ai = struct(); end
    cache_ai.iter = k;
    elapsed = toc(t0);
    S = struct('kind','intersection', 'agent', agent_i, ...
        'valid_systems', valid_systems, 'best_x', best_x, 'best_y', best_y, ...
        'best_alpha', {best_alpha}, 'best_gamma', {best_gamma}, ...
        'best_idx', best_idx, 'cache', cache_ai, 'elapsed', elapsed, 'worker', wid);
end

function S = agent_update_road_single(const, agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, xi_prev_bar, yi_prev_bar, ai_x, ai_y)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    [x_road, y_road] = updateRoadAgent(agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, xi_prev_bar, yi_prev_bar, ai_x, ai_y, const);
    elapsed = toc(t0);
    S = struct('kind','road', 'agent', agent_i, 'valid_systems', valid_systems, ...
        'x_road', x_road, 'y_road', y_road, 'elapsed', elapsed, 'worker', wid);
end

function S = agent_update_terminal_single(const, x_prev9, x_prev_bar9, a_x_new9)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    [x9_new, delay_cost] = updateAgent9(x_prev9, x_prev_bar9, a_x_new9, const);
    elapsed = toc(t0);
    S = struct('kind','terminal', 'x9_new', x9_new, 'delay_cost', delay_cost, ...
        'elapsed', elapsed, 'worker', wid);
end

function S = agent_update_intersection_stub_single(agent_i)
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    S = struct('kind','intersection', 'agent', agent_i, ...
        'valid_systems', [], 'best_x', [], 'best_y', [], ...
        'best_alpha', {{}}, 'best_gamma', {{}}, 'best_idx', [], 'cache', [], ...
        'elapsed', 0, 'worker', wid);
end

function S = agent_update_road_stub_single(agent_i)
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end
    S = struct('kind','road', 'agent', agent_i, ...
        'valid_systems', [], 'x_road', [], 'y_road', [], ...
        'elapsed', 0, 'worker', wid);
end
