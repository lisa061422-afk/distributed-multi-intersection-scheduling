%% MAIN_Sequential_Debug.m
% Same parameters as MAIN_Parallel_Compute, but NO parallel pool.
% Agents run sequentially — use this for debugging decision-tree behaviour.
clear all;

% ===================== Mode switch =====================
configMode = 'manual';

% ===================== Physical parameters =====================
T_val        = 2.0;
T_ent        = 0.0;
Dt           = 2.0;
v_max_phys   = 1.5;
W            = 1.6;
detect_range_val = 7.6;
paths = renke_project_paths();
DEMO_DIR = paths.demoDir;

% ===================== Batch settings =====================
numVehiclesList = [10];
seedList        = [0];

demoScene   = 'S1';
rootSaveDir = paths.batchDir;

% ===================== Run Mode =====================
runMode         = 'normal';     % 'normal' or 'priority'
priority_robots = [6];

skip_normal_run       = strcmp(runMode, 'priority');
enable_priority_sweep = strcmp(runMode, 'priority');

if ~exist(rootSaveDir, 'dir'), mkdir(rootSaveDir); end

%% ─── NO parpool ────────────────────────────────────────────────────────────

for iN = 1:numel(numVehiclesList)
for iS = 1:numel(seedList)

close all;
Nveh = numVehiclesList(iN);
seed = seedList(iS);
caseName = 'manual_10r';

caseDir = fullfile(rootSaveDir, caseName);
if ~exist(caseDir,'dir'), mkdir(caseDir); end

fprintf('\n=== Sequential Debug: %s ===\n', caseName);

%% ADMM parameters
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 50;
tol_r = 5e-3; tol_s = 5e-3;
randInitScale = 0;

%% Intersection DB
IntSpaceDB   = makeIntSpaceDB();
LocalTreeCache = cell(9,1);

%% Config (manual)
config = {
    struct('entrance',3,'exits',[2]),
    struct('entrance',3,'exits',[5]),
    struct('entrance',5,'exits',[6]),
    struct('entrance',5,'exits',[8]),
    struct('entrance',1,'exits',[3]),
    struct('entrance',6,'exits',[8]),
    struct('entrance',6,'exits',[1]),
    struct('entrance',2,'exits',[5]),
    struct('entrance',2,'exits',[8]),
    struct('entrance',4,'exits',[7])
};
vehicleList = [];
stats       = struct();
N = numel(config);

%% Routes / timing (copy from MAIN_Parallel_Compute)
for n = 1:N
    config{n}.entryIndex = 1;
end
entCounts = zeros(1,8);
for n = 1:N
    e = config{n}.entrance;
    entCounts(e) = entCounts(e) + 1;
    config{n}.entryIndex = entCounts(e);
end

pathInfo = getVehiclePaths(config);

% Build the same agent chains, traversal durations, and participation map
% used by MAIN_Parallel_Compute. getVehiclePaths only resolves routes.
pathInfo_agent_chain = cell(1,N);
pathInfo_c = cell(1,N);
for n = 1:N
    kn = 1;
    pathInfo_agent_chain{n} = cell(1,kn);
    pathInfo_c{n} = cell(1,kn);

    int_seq  = pathInfo{n}(kn).int;
    routeIds = pathInfo{n}(kn).routeId;
    dur = zeros(1,numel(int_seq));
    for k_int = 1:numel(int_seq)
        ag  = int_seq(k_int);
        rId = routeIds(k_int);
        dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});
    end

    ag_chain = [];
    for i = 1:length(int_seq)-1
        ag_chain = [ag_chain, int_seq(i), ...
            getRoadAgent(int_seq(i), int_seq(i+1))]; %#ok<AGROW>
    end
    ag_chain = [ag_chain, int_seq(end), 9];
    pathInfo_agent_chain{n}{kn} = ag_chain;
    pathInfo_c{n}{kn} = dur;
end

agent_participation = repmat({cell(N,1)},8,1);
for n = 1:N
    chain = pathInfo_agent_chain{n}{1};
    for ag = chain(1:end-1)
        agent_participation{ag}{n} = 1;
    end
end

v_max          = ones(1,N) * v_max_phys;
detect_range   = ones(1,N) * detect_range_val;
initial_position = cell(1,N);
for n = 1:N
    initial_position{n} = config{n}.entrance;
end

headway = T_val;
ent_rank = zeros(1,8);

alpha_tilde = cell(N,1);
speed       = cell(N,1);
for n = 1:N, speed{n} = 0; end

deadline = cell(N,1);
for n = 1:N
    kn   = 1;
    base = (detect_range(n)/2 - W/2) / v_max(n);
    alpha_tilde{n}    = zeros(1,1);
    alpha_tilde{n}(1) = base + ent_rank(config{n}.entrance)*T_ent ...
                             + (config{n}.entryIndex - 1)*headway;
    durations  = pathInfo_c{n}{kn};
    c_total    = sum(durations);
    chain      = pathInfo_agent_chain{n}{kn};
    num_roads  = floor((length(chain)-1)/2);
    deadline{n}    = zeros(kn,1);
    deadline{n}(kn) = alpha_tilde{n}(kn) + c_total + Dt*num_roads;
end

%% Pack const
const = struct();
const.rho1   = rho1;  const.rho2   = rho2;
const.weight = weight; const.max_iter = max_iter;
const.tol_r  = tol_r; const.tol_s  = tol_s;
const.N      = N;
const.Dt     = Dt;
const.priority_n       = 0;
const.deadline         = deadline;
const.alpha_tilde      = alpha_tilde;
const.initial_position = initial_position;
const.config           = config;
const.pathInfo         = pathInfo;
const.pathInfo_agent_chain = pathInfo_agent_chain;
const.pathInfo_c       = pathInfo_c;
const.agent_participation = agent_participation;
const.IntSpaceDB       = IntSpaceDB;
const.randInitScale    = randInitScale;

demoGroup = sprintf('%dr', Nveh);
matFile   = fullfile(caseDir, sprintf('FourIntersection_ADMM_%s.mat', caseName));

if ~skip_normal_run
    fprintf('--- Sequential normal run ---\n');
    [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, delay_costs, k, T_ADMM_TOTAL] = ...
        run_admm_seq(const, agent_participation);
    x_hist = {};
    save(matFile, 'config','vehicleList','stats','const', ...
        'residual_r','residual_s','x_hist','x_prev','y_prev', ...
        'max_iter','k','delay_costs','LocalTreeCache','T_ADMM_TOTAL','seed','Nveh');
end

%% Priority sweep
if enable_priority_sweep && ~isempty(priority_robots)
    if ~exist('const','var') || ~isfield(const,'rho1')
        load(matFile, 'x_prev','y_prev','LocalTreeCache','residual_r','residual_s','delay_costs','k','const');
    end
    const_base = const;

    for prio_n = priority_robots
        fprintf('\n--- Priority Robot %d (sequential) ---\n', prio_n);
        const_p            = const_base;
        const_p.priority_n = prio_n;

        [xp, yp, LCp, rr_p, rs_p, dc_p, kp, Tp] = ...
            run_admm_seq(const_p, agent_participation);

        caseName_p = sprintf('%s_pR%d', caseName, prio_n);
        caseDir_p  = fullfile(rootSaveDir, caseName_p);
        if ~exist(caseDir_p,'dir'), mkdir(caseDir_p); end

        const_save     = const_p;
        x_prev_save    = xp;  y_prev_save = yp;
        LC_save        = LCp;
        matFile_p = fullfile(caseDir_p, sprintf('FourIntersection_ADMM_%s.mat', caseName_p));
        save(matFile_p, 'const_save','x_prev_save','y_prev_save','LC_save', ...
            'rr_p','rs_p','dc_p','kp','Tp','seed','Nveh');
        fprintf('Saved: %s\n', matFile_p);
    end
end

end % iS
end % iN


%% ═══════════════════════ Sequential ADMM core ════════════════════════════
function [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, ...
          delay_costs, k, T_ADMM] = run_admm_seq(const, agent_participation)

N        = const.N;
Dt       = const.Dt;
max_iter = const.max_iter;
tol_r    = const.tol_r;
tol_s    = const.tol_s;
rho1     = const.rho1;

LocalTreeCache = cell(9,1);
[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(const.alpha_tilde, const.pathInfo_agent_chain, ...
                                  const.pathInfo_c, Dt, N, const.randInitScale);

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

residual_r  = zeros(max_iter,1);
residual_s  = zeros(max_iter,1);
delay_costs = zeros(max_iter,1);
T0 = tic;

for k = 1:max_iter
    fprintf('k = %d\n', k);
    t_iter = tic;
    x_last = x_prev; y_last = y_prev;

    %% Step 1: dual variable update (identical to parallel version)
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
            prev_ag = chain(pos-1);
            curr_ag = chain(pos);
            ax_loc{curr_ag}(kn) = a_x{curr_ag}{n}(kn) + ...
                rho1*(x_prev{curr_ag}{n}(kn) - y_prev{prev_ag}{n}(kn));
            xbar_loc{curr_ag}(kn) = ...
                (x_prev{curr_ag}{n}(kn) + y_prev{prev_ag}{n}(kn)) / 2;
            ay_loc{prev_ag}(kn) = a_y{prev_ag}{n}(kn) + ...
                rho1*(y_prev{prev_ag}{n}(kn) - x_prev{curr_ag}{n}(kn));
            ybar_loc{prev_ag}(kn) = ...
                (y_prev{prev_ag}{n}(kn) + x_prev{curr_ag}{n}(kn)) / 2;
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

    %% Step 2: sequential local updates (no parfeval)
    meta = cell(1,9);
    for agent_i = 1:9
        t0 = tic;
        if agent_i >= 1 && agent_i <= 4
            entries       = agent_participation{agent_i};
            valid_systems = find(~cellfun(@isempty, entries))';
            if isempty(valid_systems)
                meta{agent_i} = struct('kind','intersection','agent',agent_i, ...
                    'valid_systems',[],'best_x',[],'best_y',[], ...
                    'best_alpha',{{}},'best_gamma',{{}},'best_idx',[], ...
                    'cache',[],'elapsed',0,'worker',-1);
            else
                [best_x, best_y, best_alpha, best_gamma, best_idx, ~, cache_out] = ...
                    INi_Admm_DecisionTree(agent_i, entries, ...
                        x_prev, y_prev, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        valid_systems, a_x_new{agent_i}, a_y_new{agent_i}, ...
                        const, LocalTreeCache{agent_i});
                if isempty(cache_out) || ~isstruct(cache_out), cache_out = struct(); end
                cache_out.iter = k;
                meta{agent_i} = struct('kind','intersection','agent',agent_i, ...
                    'valid_systems',valid_systems,'best_x',best_x,'best_y',best_y, ...
                    'best_alpha',{best_alpha},'best_gamma',{best_gamma},'best_idx',best_idx, ...
                    'cache',cache_out,'elapsed',toc(t0),'worker',-1);
            end

        elseif agent_i >= 5 && agent_i <= 8
            entries       = agent_participation{agent_i};
            valid_systems = find(~cellfun(@isempty, entries))';
            if isempty(valid_systems)
                meta{agent_i} = struct('kind','road','agent',agent_i, ...
                    'valid_systems',[],'x_road',[],'y_road',[],'elapsed',0,'worker',-1);
            else
                [x_road, y_road] = updateRoadAgent(agent_i, entries, valid_systems, ...
                    x_prev{agent_i}, y_prev{agent_i}, ...
                    x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                    a_x_new{agent_i}, a_y_new{agent_i}, const);
                meta{agent_i} = struct('kind','road','agent',agent_i, ...
                    'valid_systems',valid_systems,'x_road',x_road,'y_road',y_road, ...
                    'elapsed',toc(t0),'worker',-1);
            end

        else  % agent 9 — terminal
            [x9_new, delay_cost] = updateAgent9(x_prev{9}, x_prev_bar{9}, a_x_new{9}, const);
            meta{agent_i} = struct('kind','terminal','x9_new',x9_new, ...
                'delay_cost',delay_cost,'elapsed',toc(t0),'worker',-1);
        end
        fprintf('  Agent %d  %.3fs\n', agent_i, toc(t0));
    end

    %% Merge results (identical to parallel version)
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
                    r_local = r_local + (S_res.best_x(n) - S_res.best_alpha{n}(kn))^2 ...
                                      + (S_res.best_y(n) - S_res.best_gamma{n}(kn))^2;
                end
            case 'road'
                for n = S_res.valid_systems
                    kn = 1;
                    x_prev{agent_i}{n}(kn) = S_res.x_road(n);
                    y_prev{agent_i}{n}(kn) = S_res.y_road(n);
                end
            case 'terminal'
                delay_costs(k) = S_res.delay_cost;
                for n = 1:N
                    x_prev{9}{n}(1) = S_res.x9_new(n);
                end
        end
    end

    %% Step 3: residuals
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
    for n = 1:N
        s = s + norm(x_prev{9}{n} - x_last{9}{n})^2;
    end

    fprintf('r=%.6f  s=%.6f\n', r, s);
    residual_r(k) = r;
    residual_s(k) = s;
    a_x = a_x_new; a_y = a_y_new;

    if r < tol_r && s < tol_s
        fprintf('Converged at iteration %d\n', k);
        residual_r  = residual_r(1:k);
        residual_s  = residual_s(1:k);
        delay_costs = delay_costs(1:k);
        break;
    end
    fprintf('[Iter %d] total time = %.3f s\n', k, toc(t_iter));
end

T_ADMM = toc(T0) / 60;
fprintf('Sequential ADMM elapsed %.3f mins  (priority_n=%d)\n', T_ADMM, const.priority_n);
end
