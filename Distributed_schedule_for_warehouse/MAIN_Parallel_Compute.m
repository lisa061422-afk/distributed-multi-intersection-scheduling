%Main program for multi-traffic intersection with c-admm
clear all;
addpath('C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Centralized\Centralized_FCFS_031426');

% ===================== Mode switch =====================
% 'manual' : fixed 10-robot config (warehouse demo special case)
% 'random' : generateBalancedTrafficConfig batch runs
configMode = 'random'; %'manual';

% ===================== Physical parameters =====================
T_val        = 2.0;    % minimum headway between two cars at same entrance (s)
Dt           = 2.0;    % road travel time between consecutive intersections (s)
v_max_phys   = 1.5;    % AMR speed on road (m/s)
W            = 1.6;    % merging zone width (m)  — scale 1:12.5 from traffic W=20m
detect_range_val = 7.6; % detection range (m): road(3m) + half-zone(0.8m), on each side
                        % => alpha_tilde base = (7.6/2 - 1.6/2) / 1.5 = 2.0 s

DEMO_DIR     = 'C:\Users\robin\OneDrive\Documents\Github_file\Traffic_Demo\schedules';
% FCFS functions are now in this folder (copied from Centralized_FCFS_031426).

% ===================== Batch settings =====================
if strcmp(configMode, 'manual')
    numVehiclesList = [10];
    seedList        = [0];   % seed unused in manual mode
else
    numVehiclesList = [10];
    seedList        = [41206];
end

% ===================== Demo export settings (random mode only) =====================
% demoScene: HTML scene label for this run — change each time you run a new seed
% demoGroup is auto-derived from Nveh (e.g. 10 robots → '10r')
demoScene = 'S1';   % <-- change to 'S2', 'S3', etc. for each new seed
rootSaveDir = 'BatchRuns';

if ~exist(rootSaveDir, 'dir')
    mkdir(rootSaveDir);
end

% ===================== Parallel pool (10 workers) =====================
if license('test','Distrib_Computing_Toolbox')
    p = gcp('nocreate');
    if isempty(p) || p.NumWorkers ~= 10
        if ~isempty(p)
            delete(p);
        end
        parpool('local', 10);
    end
end

%----------------------------------------------------------------------
for iN = 1:numel(numVehiclesList)
    for iS = 1:numel(seedList)

        close all;
        Time_begin = tic;

        Nveh = numVehiclesList(iN);
        seed = seedList(iS);

        if strcmp(configMode, 'manual')
            caseName = 'manual_10r';
        else
            caseName = sprintf('seed_%d_N_%d', seed, Nveh);
        end

        caseDir = fullfile(rootSaveDir, caseName);
        if ~exist(caseDir, 'dir')
            mkdir(caseDir);
        end

        fprintf('\n====================================================\n');
        fprintf('Running case: %s  [configMode=%s]\n', caseName, configMode);
        fprintf('====================================================\n');

%% -----------------------ADMM penalty parameters-------------------------
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 300;
tol_r = 5e-3; tol_s = 5e-3;

%% Local Intersection Information
IntSpaceDB = makeIntSpaceDB();
LocalTreeCache = cell(9,1);

%% Config generation
if strcmp(configMode, 'manual')
    % ── Fixed 10-robot warehouse demo config ──────────────────────────
    config_raw = {
        struct('entrance', 1, 'exits', [7 3]),
        struct('entrance', 4, 'exits', [1 3]),
        struct('entrance', 6, 'exits', [1]),
        struct('entrance', 7, 'exits', [4]),
        struct('entrance', 5, 'exits', [3]),
        struct('entrance', 3, 'exits', [2 4]),
        struct('entrance', 2, 'exits', [5]),
    };
    vehicleList = [];
    stats = struct();
else
    % ── Random balanced config ─────────────────────────────────────────
    [config_raw, vehicleList, stats] = generateBalancedTrafficConfig(Nveh, ...
        'Seed', seed, ...
        'MaxPerEntrance', 3, ...
        'EntrancePenalty', 0.4);
    configFile = fullfile(caseDir, sprintf('config_seed%d_N%d.m', seed, Nveh));
    printTrafficConfig(config_raw, configFile);
end

% ---- expand route-based config → vehicle-based config ----
vehicleConfig = {};
vid = 0;
for g = 1:length(config_raw)
    ent  = config_raw{g}.entrance;
    exits = config_raw{g}.exits;
    for j = 1:length(exits)
        vid = vid + 1;
        vehicleConfig{vid} = struct( ...
            'entrance',   ent, ...
            'exits',      exits(j), ...
            'entryIndex', j ...
        );
    end
end
config = vehicleConfig;
Nveh   = length(config);   % actual vehicle count after expansion

%% Routing
% NOTE: pathInfo_c is derived directly from IntSpaceDB.routeDur (sum per intersection).
% Do NOT use a separate hardcoded cTime — changes to makeIntSpaceDB automatically propagate.
N = length(config);
pathInfo = getVehiclePaths(config);

pathInfo_agent_chain = cell(1,N);
pathInfo_c = cell(1,N);
for n = 1:N
    kn = 1;
    pathInfo_agent_chain{n} = cell(1, kn);
    pathInfo_c{n}           = cell(1, 1);
    int_seq  = pathInfo{n}(kn).int;
    routeIds = pathInfo{n}(kn).routeId;   % per-intersection route ID

    % Derive total traversal time for each intersection from IntSpaceDB
    dur = zeros(1, numel(int_seq));
    for k_int = 1:numel(int_seq)
        ag  = int_seq(k_int);
        rId = routeIds(k_int);
        dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});  % sum of space-level C values
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

%% agent_participation
agent_participation = repmat({cell(N, 1)}, 8, 1);
for n = 1:N
    kn    = 1;
    chain = pathInfo_agent_chain{n}{kn};
    ags   = chain(1:end-1);
    for ii = 1:numel(ags)
        ag = ags(ii);
        agent_participation{ag}{n} = 1;
    end
end

%% Physical setup
v_max = v_max_phys * ones(1,N);
d1    = zeros(1,N);
detect_range = detect_range_val * ones(1,N);

headway = T_val;   % same-entrance headway

init_p_vehi_1    = zeros(N,1);
initial_position = zeros(N,1);
alpha_tilde      = cell(N,1);

% Special case: vehicles 3 and 5 arrive 0.3s earlier (proportional to traffic LCSS example)
% traffic: detect_range(n) = 510 - 2*0.3*v_max(n) => alpha_tilde shifts by -0.3s
% warehouse: same formula, detect_range(n) = 7.6 - 2*0.3*v_max(n) => same -0.3s shift
% if strcmp(configMode, 'manual') && N >= 5
%     detect_range(3) = detect_range_val - 2 * 0.3 * v_max(3);
%     detect_range(5) = detect_range_val - 2 * 0.3 * v_max(5);
% end

for n = 1:N
    alpha_tilde{n} = zeros(1,1);
    base = (detect_range(n)/2 - W/2) / v_max(n) + d1(n);
    alpha_tilde{n}(1) = base + (config{n}.entryIndex - 1) * headway;

    t_arr = alpha_tilde{n}(1);
    init_p_vehi_1(n)    = -(detect_range(n)/2 - W/2 + t_arr * v_max(n));
    initial_position(n) = init_p_vehi_1(n);
end

speed = cell(N,1);
for n = 1:N, speed{n} = 0; end

%% Earliest Exit Times (deadline)
deadline = cell(N, 1);
for n = 1:N
    kn = 1;
    assert(numel(alpha_tilde{n}) == 1, 'Expected alpha_tilde{n} length = 1.');
    deadline{n}   = zeros(kn, 1);
    durations     = pathInfo_c{n}{kn};
    c_total       = sum(durations);
    chain         = pathInfo_agent_chain{n}{kn};
    num_roads     = floor((length(chain) - 1) / 2);
    deadline{n}(kn) = alpha_tilde{n}(kn) + (c_total + Dt * num_roads);
end

%% Pack constants
const = struct();
const.rho1   = rho1;  const.rho2   = rho2;
const.weight = weight; const.max_iter = max_iter;
const.tol_r  = tol_r; const.tol_s  = tol_s;
const.N      = N;
const.Dt     = Dt;
const.deadline     = deadline;
const.alpha_tilde  = alpha_tilde;
const.initial_position = initial_position;
const.config       = config;
const.pathInfo     = pathInfo;
const.pathInfo_agent_chain = pathInfo_agent_chain;
const.pathInfo_c   = pathInfo_c;
const.agent_participation = agent_participation;
const.IntSpaceDB   = IntSpaceDB;

%% ADMM initialisation
[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N);

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
x_hist = cell(1, 9);
for i = 1:9
    x_hist{i} = cell(1, N);
    for n = 1:N
        K = length(alpha_tilde{n});
        x_hist{i}{n} = NaN(K, max_iter);
    end
end

iter_time     = zeros(max_iter,1);
agent_time    = NaN(max_iter,9);
agent_worker  = NaN(max_iter,9);
worker_time   = zeros(max_iter, 10);

%%  ──────────────────────── ADMM main loop ──────────────────────────────
N = const.N; max_iter = const.max_iter;
tol_r = const.tol_r; tol_s = const.tol_s; rho1 = const.rho1;

for k = 1 : max_iter
    k
    t_iter = tic;
    x_last = x_prev; y_last = y_prev; r_local = 0;

    %% Step 1: update dual variables
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
            if ag <= 8
                ybar_loc{ag} = y_prev_bar{ag}{n};
            end
        end

        ag0 = chain(1);
        xbar_loc{ag0}(kn) = (x_prev{ag0}{n}(kn) + 0) / 2;

        for pos = 2:length(chain)
            prev_ag = chain(pos - 1);
            curr_ag = chain(pos);

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

    %% Step 2: parallel local updates
    f    = parallel.FevalFuture.empty(0,9);
    meta = cell(1,9);

    for agent_i = 1:9
        if agent_i >= 1 && agent_i <= 4
            entries       = agent_participation{agent_i};
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

    meta = cell(1,9);
    done_flags = false(1,9);
    for ii = 1:9
        try
            [completedIdx, S] = fetchNext(f);
            meta{completedIdx}       = S;
            done_flags(completedIdx) = true;
        catch ME
            fprintf(2, '\n[Iter %d] fetchNext error:\n%s\n', ...
                k, getReport(ME,'extended','hyperlinks','off'));
            rethrow(ME);
        end
    end

    %% Merge results
    r_local = 0;
    for agent_i = 1:9
        S = meta{agent_i};
        fprintf('Agent %d time=%.3f worker=%d\n', agent_i, S.elapsed, S.worker);
        switch S.kind
            case 'intersection'
                LocalTreeCache{agent_i} = S.cache;
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
    r = compute_r(x_prev, y_prev, r_local, const);
    s = 0;
    for agent_i = 1:8
        entries = agent_participation{agent_i};
        if all(cellfun(@isempty, entries)), continue; end
        valid_systems = find(~cellfun(@isempty, entries))';
        for n = valid_systems
            s = s + norm(x_prev{agent_i}{n} - x_last{agent_i}{n})^2 + ...
                    norm(y_prev{agent_i}{n} - y_last{agent_i}{n})^2;
        end
    end
    for n = 1:N
        s = s + norm(x_prev{9}{n} - x_last{9}{n})^2;
    end

    r, s
    residual_r(k) = r; residual_s(k) = s;

    for agent_i = 1:9
        for n = 1:N
            x_hist{agent_i}{n}(1,k) = x_prev{agent_i}{n}(1);
        end
    end

    a_x = a_x_new; a_y = a_y_new;

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

%% Save distributed (ADMM) result
T_ADMM_TOTAL = toc(Time_begin)/60;
fprintf('ADMM elapsed %.3f mins\n', T_ADMM_TOTAL);

caseConfigFile = fullfile(caseDir, 'case_config.mat');
save(caseConfigFile, ...
    'config', 'seed', 'Nveh', 'T_val', ...
    'detect_range', 'v_max', ...
    'alpha_tilde', 'initial_position', ...
    'pathInfo', 'pathInfo_agent_chain', 'pathInfo_c');

matFile = fullfile(caseDir, sprintf('FourIntersection_ADMM_%s.mat', caseName));
save(matFile, ...
    'config', 'vehicleList', 'stats', ...
    'const', ...
    'residual_r', 'residual_s', ...
    'x_hist', ...
    'x_prev', 'y_prev', ...
    'max_iter', 'k', 'delay_costs', 'LocalTreeCache', ...
    'T_ADMM_TOTAL', 'seed', 'Nveh');

load(matFile);

%% Plots – distributed schedule
figs_before = findall(0, 'Type', 'figure');
plot_C_ADMM2(residual_r, residual_s, delay_costs, ...
    x_prev, y_prev, const.pathInfo_agent_chain, N, k);
drawnow;
figs_after = findall(0, 'Type', 'figure');
new_figs = setdiff(figs_after, figs_before);
[~, ord] = sort(arrayfun(@(f) f.Number, new_figs));
new_figs = new_figs(ord);
fig_macro = new_figs(min(3, end));   % Figure 3 = intersection schedule

macroBase    = fullfile(caseDir, sprintf('macro_schedule_%s', caseName));
macroFigFile = [macroBase '.png'];
savefig(fig_macro, [macroBase '.fig']);
print(fig_macro, macroFigFile, '-dpng', '-r300');

fig = figure('Color', 'w', 'Position', [60 40 1500 950]);
panelPos = {
    [0.04 0.53 0.44 0.42],
    [0.52 0.53 0.44 0.42],
    [0.04 0.05 0.44 0.42],
    [0.52 0.05 0.44 0.42]
};
for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    if isempty(cache), continue; end
    p = uipanel('Parent', fig, ...
        'Units', 'normalized', ...
        'Position', panelPos{agent_i}, ...
        'BackgroundColor', [0.97 0.97 0.97], ...
        'BorderType', 'none');
    plot_local_schedule_final_into_panel_1(p, ...
        cache.NODES, cache.Path, agent_i, cache.valid_systems, const, ...
        'x_prev', x_prev, ...
        'gap', 0.004, 'panelColor', 'w', 'axColor', [0.98 0.98 0.98], ...
        'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0.06);
end
drawnow; pause(0.3);
localBase    = fullfile(caseDir, sprintf('local_schedules_%s', caseName));
localPngFile = [localBase '.png'];
savefig(fig, [localBase '.fig']);
print(fig, localPngFile, '-dpng', '-r200');

%% ══════════════════════════════════════════════════════════════════════════
%%  FCFS (Centralized Branch-and-Bound) using the SAME const parameters
%% ══════════════════════════════════════════════════════════════════════════
fprintf('\n===== Running FCFS with same config/timing =====\n');

% Build const_fcfs from distributed const (guarantees identical alpha_tilde & Dt)
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

NI_fcfs = zeros(1, N);
for n = 1:N, NI_fcfs(n) = Veh_fcfs(n).NI; end

const_fcfs              = struct();
const_fcfs.N            = N;
const_fcfs.Dt           = const.Dt;
const_fcfs.Veh          = Veh_fcfs;
const_fcfs.IntSpaceDB   = const.IntSpaceDB;
const_fcfs.alpha_tilde  = const.alpha_tilde;
const_fcfs.Smax         = 3;
const_fcfs.numInt       = 4;
const_fcfs.spacePerInt  = 5;
const_fcfs.Mtot         = 4 * 5;
const_fcfs.BIG_M        = 1000;
const_fcfs.NI           = NI_fcfs;

% FCFS BnB root node
Smax_f  = const_fcfs.Smax;
Mtot_f  = const_fcfs.Mtot;
d0_f    = inf(1,N);
r0_f    = zeros(Smax_f, N);
o0_f    = zeros(1,N);
ni0_f   = zeros(1,N);
U_c0_f  = zeros(N, Mtot_f);
U0_f    = zeros(N, Mtot_f);

gamma0_f     = cell(1,N);
alpha0_node_f = cell(1,N);
x0_f         = cell(N,1);
speed0_f     = cell(1,N);
ra_reset0_f  = zeros(Smax_f, N);

for n = 1:N
    d0_f(n)           = Veh_fcfs(n).alpha0;
    gamma0_f{n}       = NaN(1, NI_fcfs(n));
    alpha0_node_f{n}  = NaN(1, NI_fcfs(n));
    x0_f{n}           = cell(1, NI_fcfs(n));
    speed0_f{n}       = 0;
end

NODES_fcfs = {{1, d0_f, r0_f, o0_f, 0, ni0_f, 0, U_c0_f, U0_f, 0, ...
               gamma0_f, 0, speed0_f, ra_reset0_f, x0_f, alpha0_node_f}};
OPEN_fcfs  = 1;
LEAF_fcfs  = [];
step_fcfs  = 0;
max_steps_fcfs = 50000;

Time_fcfs = tic;
while ~isempty(OPEN_fcfs)
    step_fcfs = step_fcfs + 1;
    if step_fcfs > max_steps_fcfs
        error('FCFS: Exceeded max_expand_steps (%d).', max_steps_fcfs);
    end
    [~, minIdx_fcfs]     = f_min2(NODES_fcfs, OPEN_fcfs);
    [NODES_fcfs, OPEN_fcfs, LEAF_fcfs] = expand_array_global2( ...
        NODES_fcfs, OPEN_fcfs, minIdx_fcfs, LEAF_fcfs, const_fcfs);
end
T_FCFS = toc(Time_fcfs);
fprintf('FCFS elapsed %.2f s  (%d steps)\n', T_FCFS, step_fcfs);

[Path_fcfs, Cost_fcfs, leaf_fcfs] = extract_optimal_path_from_nodes( ...
    NODES_fcfs, LEAF_fcfs, 'criterion', 'g_min');
DATA_fcfs = build_centralized_schedule_data(NODES_fcfs, Path_fcfs, const_fcfs);

% Save FCFS result
fcfsMatFile = fullfile(caseDir, sprintf('FCFS_%s.mat', caseName));
save(fcfsMatFile, 'const_fcfs', 'DATA_fcfs', 'seed', 'Nveh', 'Cost_fcfs');
fprintf('FCFS saved: %s  (cost=%.4f)\n', fcfsMatFile, Cost_fcfs);

% Plot FCFS schedule — same figure size / style as distributed local panel
fig_fcfs = figure('Color', 'w', 'Position', [60 40 1500 950]);
panelPos_fcfs = {
    [0.04 0.53 0.44 0.42],
    [0.52 0.53 0.44 0.42],
    [0.04 0.05 0.44 0.42],
    [0.52 0.05 0.44 0.42]};
for agent_i = 1:4
    vs_fcfs = DATA_fcfs.valid_systems{agent_i};
    if isempty(vs_fcfs), continue; end
    pf = uipanel('Parent', fig_fcfs, ...
        'Units', 'normalized', 'Position', panelPos_fcfs{agent_i}, ...
        'BackgroundColor', [0.97 0.97 0.97], 'BorderType', 'none');
    plot_local_schedule_final_into_panel_centralized(pf, DATA_fcfs, const_fcfs, ...
        agent_i, vs_fcfs, ...
        'gap', 0.004, 'panelColor', 'w', 'axColor', [0.98 0.98 0.98], ...
        'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0.06);
end
drawnow; pause(0.3);
fcfsBase    = fullfile(caseDir, sprintf('fcfs_schedule_%s', caseName));
fcfsPngFile = [fcfsBase '.png'];
savefig(fig_fcfs, [fcfsBase '.fig']);
print(fig_fcfs, fcfsPngFile, '-dpng', '-r200');

%% Export JS files for HTML demo (manual mode only)
if strcmp(configMode, 'manual')
    % ADMM optimal result
    export_demo_json(const, x_prev, y_prev, ...
        '10r · Warehouse', ...
        fullfile(DEMO_DIR, '10r_S1_optimal.js'), ...
        'optimal');

    % FCFS result
    export_demo_json_fcfs(const_fcfs, DATA_fcfs, ...
        '10r · Warehouse', ...
        fullfile(DEMO_DIR, '10r_S1_fcfs.js'), ...
        'fcfs');

    fprintf('Demo JS files exported to:\n  %s\n', DEMO_DIR);

    % ── Copy schedule PNGs to schedules/images/ for HTML viewer ─────────
    img_dir = fullfile(DEMO_DIR, 'images');
    if ~exist(img_dir, 'dir'), mkdir(img_dir); end

    % HTML naming: {group}_{scene}_{policy}_{tag}.png
    % robotGroup='10r', sceneName='S1' in warehouse_amr_demo_test.html
    copyfile(macroFigFile,  fullfile(img_dir, '10r_S1_optimal_macro.png'));
    copyfile(localPngFile,  fullfile(img_dir, '10r_S1_optimal_local.png'));
    copyfile(fcfsPngFile,   fullfile(img_dir, '10r_S1_fcfs_local.png'));
    fprintf('Schedule PNGs copied to: %s\n', img_dir);

    % ── Export decision-tree JSON for HTML renderer ──────────────────────
    treeJsFile = fullfile(DEMO_DIR, 'tree_10r_S1_optimal.js');
    export_tree_json(LocalTreeCache, '10r', 'S1', treeJsFile);
end

%% ── Auto-export to HTML demo (random mode) ──────────────────────────────
if strcmp(configMode, 'random')
    demoGroup = sprintf('%dr', Nveh);   % e.g. 10 → '10r'
    fprintf('\nAuto-exporting to demo: group=%s  scene=%s\n', demoGroup, demoScene);
    export_scenario_to_demo(caseName, demoGroup, demoScene);
end

fprintf('Finished case: %s\n', caseName);
fprintf('Saved to: %s\n', caseDir);
    end  % iS
end  % iN

%% ═══════════════════════ Local agent functions ═══════════════════════════

function S = agent_update_intersection_single(const, agent_i, entries, valid_systems, ...
        x_prev_all, y_prev_all, xi_prev_bar, yi_prev_bar, ai_x, ai_y, cache_ai, k)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    if isempty(cache_ai) || ~isstruct(cache_ai)
        cache_ai = struct();
    end

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
    elapsed = toc(t0);

    S = struct('kind','intersection', ...
        'agent', agent_i, ...
        'valid_systems', valid_systems, ...
        'best_x', best_x, ...
        'best_y', best_y, ...
        'best_alpha', {best_alpha}, ...
        'best_gamma', {best_gamma}, ...
        'best_idx', best_idx, ...
        'cache', cache_ai, ...
        'elapsed', elapsed, ...
        'worker', wid);
end

function S = agent_update_road_single(const, agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, xi_prev_bar, yi_prev_bar, ai_x, ai_y)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    [x_road, y_road] = updateRoadAgent( ...
        agent_i, entries, valid_systems, ...
        x_prev_i, y_prev_i, ...
        xi_prev_bar, yi_prev_bar, ...
        ai_x, ai_y, const);

    elapsed = toc(t0);
    S = struct('kind','road', ...
        'agent', agent_i, ...
        'valid_systems', valid_systems, ...
        'x_road', x_road, ...
        'y_road', y_road, ...
        'elapsed', elapsed, ...
        'worker', wid);
end

function S = agent_update_terminal_single(const, x_prev9, x_prev_bar9, a_x_new9)
    t0 = tic;
    task = getCurrentTask();
    if isempty(task), wid = -1; else, wid = task.ID; end

    [x9_new, delay_cost] = updateAgent9(x_prev9, x_prev_bar9, a_x_new9, const);

    elapsed = toc(t0);
    S = struct('kind','terminal', ...
        'x9_new', x9_new, ...
        'delay_cost', delay_cost, ...
        'elapsed', elapsed, ...
        'worker', wid);
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
