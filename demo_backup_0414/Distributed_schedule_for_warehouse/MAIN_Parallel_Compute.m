%Main program for multi-traffic intersection with c-admm
clear all;
% addpath('C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Centralized\Centralized_FCFS_031426');  % 函数已复制到当前目录

% ===================== Mode switch =====================
% 'manual' : fixed 10-robot config (warehouse demo special case)
% 'random' : generateBalancedTrafficConfig batch runs
configMode = 'manual';
% ===================== Physical parameters =====================
T_val        = 2.0;    % headway between vehicles at the SAME entrance (s)
T_ent        = 0.0;    % stagger between first vehicles at DIFFERENT entrances (s)
Dt           = 2.0;    % road travel time between consecutive intersections (s)
v_max_phys   = 1.5;    % AMR speed on road (m/s)
W            = 1.6;    % merging zone width (m)  — scale 1:12.5 from traffic W=20m
detect_range_val = 7.6; % detection range (m): road(3m) + half-zone(0.8m), on each side
                        % => alpha_tilde base = (7.6/2 - 1.6/2) / 1.5 = 2.0 s

DEMO_DIR     = 'C:\Users\robin\OneDrive\Documents\Github_file\demo_backup_0414\Traffic_Demo\schedules';
% FCFS functions are now in this folder (copied from Centralized_FCFS_031426).

% ===================== Batch settings =====================
if strcmp(configMode, 'manual')
    numVehiclesList = [10];
    seedList        = [0];   % seed unused in manual mode
else
    numVehiclesList = [30];
    seedList        = [41603];   % 30r S1 seed
end

% ===================== Demo export settings (random mode only) =====================
% demoScene: HTML scene label for this run — change each time you run a new seed
% demoGroup is auto-derived from Nveh (e.g. 10 robots → '10r')
demoScene = 'S6';   % <-- change to 'S2', 'S3', etc. for each new seed
rootSaveDir = 'BatchRuns';

% ===================== Run Mode =====================
% 'normal'   : full ADMM + FCFS + export (no priority)
% 'priority' : skip normal run, load existing result, run priority sweep only
runMode         = 'normal';   % ← 改这一个
priority_robots = [1];        % priority 模式下给哪些 robot 最高优先级

% ===================== Parallel switch =====================
% true  : use parfeval (fast, multi-worker)
% false : sequential direct calls (easy to set breakpoints and inspect variables)
useParallel = true;

% (derived — do not edit)
skip_normal_run       = strcmp(runMode, 'priority');
enable_priority_sweep = strcmp(runMode, 'priority');

if ~exist(rootSaveDir, 'dir')
    mkdir(rootSaveDir);
end

% ===================== Parallel pool (8 workers) =====================
if useParallel && license('test','Distrib_Computing_Toolbox')
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
            caseName = sprintf('manual_%dr_%s', Nveh, demoScene);
        else
            caseName = sprintf('seed_%d_N_%d_%s', seed, Nveh, demoScene);
        end

        caseDir = fullfile(rootSaveDir, caseName);
        if ~exist(caseDir, 'dir')
            mkdir(caseDir);
        end

        fprintf('\n====================================================\n');
        fprintf('Running case: %s  [configMode=%s]\n', caseName, configMode);
        fprintf('====================================================\n');

%% -----------------------ADMM penalty parameters-------------------------
rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 500;
tol_r = 1e-3; tol_s = 1e-3;

% ── Random initialisation switch ──────────────────────────────────────
% 0   : deterministic earliest-time init (original)
% >0  : each vehicle gets a uniform random delay in [0, randInitScale] seconds
%       added to its entire chain — any positive value is valid, e.g.:
%         0.5  — small nudge, stays close to earliest-time solution
%         2.0  — up to one road-segment worth of spread
randInitScale = 0;

%% Local Intersection Information
IntSpaceDB = makeIntSpaceDB();
LocalTreeCache = cell(9,1);

%% Config generation
if strcmp(configMode, 'manual')
    % ── manual_10r S1 original ────────────────────────────────────────
    config_raw = {
        struct('entrance', 1, 'exits', [7 3]),
        struct('entrance', 4, 'exits', [1 3]),
        struct('entrance', 6, 'exits', [1]),
        struct('entrance', 7, 'exits', [4]),
        struct('entrance', 5, 'exits', [3]),
        struct('entrance', 3, 'exits', [2 4]),
        struct('entrance', 2, 'exits', [5]),
    };

    % ── seed_41220_N_20 config, entrance 2 exit 3→5 ───────────────────
    % config_raw = {
    %     struct('entrance', 2, 'exits', [5 1]),   % was [3 1]
    %     struct('entrance', 1, 'exits', [6 2 5]),
    %     struct('entrance', 4, 'exits', [2 6]),
    %     struct('entrance', 6, 'exits', [5 1 8]),
    %     struct('entrance', 7, 'exits', [1 4]),
    %     struct('entrance', 5, 'exits', [2 6 7]),
    %     struct('entrance', 8, 'exits', [7 2]),
    %     struct('entrance', 3, 'exits', [5 8 4]),
    % };
    vehicleList = [];
    stats = struct();
else
    % ── Random balanced config ─────────────────────────────────────────
    [config_raw, vehicleList, stats] = generateBalancedTrafficConfig(Nveh, ...
        'Seed', seed, ...
        'MaxPerEntrance',    ceil(Nveh/4), ...
        'MaxPerIntersection', ceil(Nveh * 1.5 / 4) + 2, ...
        'MaxPerStage',       ceil(Nveh / 4) + 1, ...
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

% Assign each unique entrance a rank (order of first appearance) so that
% the first vehicle at each entrance is staggered by headway seconds.
% This reduces simultaneous conflicts across entrances and speeds up convergence.
all_entrances = cellfun(@(c) c.entrance, config);
unique_ents   = unique(all_entrances, 'stable');   % order of first appearance
ent_rank      = zeros(1, max(unique_ents));
for ei = 1:numel(unique_ents)
    ent_rank(unique_ents(ei)) = ei - 1;   % 0-indexed rank
end

for n = 1:N
    alpha_tilde{n} = zeros(1,1);
    base = (detect_range(n)/2 - W/2) / v_max(n) + d1(n);
    % within-entrance stagger + across-entrance stagger
    alpha_tilde{n}(1) = base ...
        + ent_rank(config{n}.entrance) * T_ent ...     % across-entrance stagger
        + (config{n}.entryIndex - 1)   * headway;      % within-entrance headway

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
const.priority_n  = 0;        % 0 = no priority override (normal run)
const.use_pruning = true;     % true = prune dominated nodes in local decision tree (faster for large N)
const.use_weak_rule = false;  % true = weak-rule priority lock (fewer branches); false = original algorithm  [S6: OFF]
const.deadline     = deadline;
const.alpha_tilde  = alpha_tilde;
const.initial_position = initial_position;
const.config       = config;
const.pathInfo     = pathInfo;
const.pathInfo_agent_chain = pathInfo_agent_chain;
const.pathInfo_c   = pathInfo_c;
const.agent_participation = agent_participation;
const.IntSpaceDB   = IntSpaceDB;
const.randInitScale = randInitScale;
const.useParallel   = useParallel;
% Physical parameters (for reproducibility)
const.T_val            = T_val;
const.T_ent            = T_ent;
const.v_max_phys       = v_max_phys;
const.W                = W;
const.detect_range_val = detect_range_val;

%% Demo export group label (needed in both branches below)
demoGroup = sprintf('%dr', Nveh);   % e.g. 10 → '10r'

%% ADMM run — normal (priority_n = 0, set above)
matFile = fullfile(caseDir, sprintf('FourIntersection_ADMM_%s.mat', caseName));

if skip_normal_run
    %% ── Skip normal ADMM: load existing result ───────────────────────────
    fprintf('--- skip_normal_run=true: loading existing matFile ---\n');
    if ~exist(matFile, 'file')
        error(['skip_normal_run=true but matFile not found:\n  %s\n' ...
               'Run with skip_normal_run=false first to generate it.'], matFile);
    end
    load(matFile, 'x_prev', 'y_prev', 'LocalTreeCache', ...
                  'residual_r', 'residual_s', 'delay_costs', 'k');
    fprintf('Loaded: %s\n', matFile);
else
    %% ── Full normal ADMM + FCFS + export ─────────────────────────────────
    fprintf('--- ADMM normal run (priority_n = 0) ---\n');
    [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, delay_costs, k, T_ADMM_TOTAL] = ...
        run_admm_core(const, agent_participation);
    T_TOTAL = toc(Time_begin) / 60;   % total wall-clock time incl. config + ADMM + FCFS
    fprintf('ADMM elapsed %.3f mins\n', T_ADMM_TOTAL);
    fprintf('Total elapsed %.3f mins\n', T_TOTAL);

    caseConfigFile = fullfile(caseDir, 'case_config.mat');
    save(caseConfigFile, ...
        'config', 'seed', 'Nveh', 'T_val', 'T_ent', ...
        'v_max_phys', 'W', 'detect_range_val', 'Dt', ...
        'detect_range', 'v_max', ...
        'alpha_tilde', 'initial_position', ...
        'pathInfo', 'pathInfo_agent_chain', 'pathInfo_c');

    x_hist  = {};
    max_iter = const.max_iter;
    save(matFile, ...
        'config', 'vehicleList', 'stats', ...
        'const', ...
        'residual_r', 'residual_s', ...
        'x_hist', ...
        'x_prev', 'y_prev', ...
        'max_iter', 'k', 'delay_costs', 'LocalTreeCache', ...
        'T_ADMM_TOTAL', 'T_TOTAL', 'seed', 'Nveh');

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

macroBase    = fullfile(caseDir, sprintf('macro_%s', caseName));
macroFigFile = [macroBase '.png'];
savefig(fig_macro, [macroBase '.fig']);
exportgraphics(fig_macro, macroFigFile, 'Resolution', 300);

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
        'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0);
end
drawnow; pause(0.3);
localBase    = fullfile(caseDir, sprintf('local_%s', caseName));
localPngFile = [localBase '.png'];
savefig(fig, [localBase '.fig']);
print(fig, localPngFile, '-dpng', '-r150');

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
        'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0);
end
drawnow; pause(0.3);
    fcfsBase    = fullfile(caseDir, sprintf('fcfs_%s', caseName));
    fcfsPngFile = [fcfsBase '.png'];
    savefig(fig_fcfs, [fcfsBase '.fig']);
    print(fig_fcfs, fcfsPngFile, '-dpng', '-r150');

    %% ── Auto-export to HTML demo ─────────────────────────────────────────
    fprintf('\nAuto-exporting to demo: group=%s  scene=%s\n', demoGroup, demoScene);
    export_scenario_to_demo(caseName, demoGroup, demoScene);

end  % if ~skip_normal_run

%% ── Priority Override ────────────────────────────────────────────────────
if enable_priority_sweep && ~isempty(priority_robots)
    fprintf('\n========================================================\n');
    fprintf('Priority Override: robots %s, scene base = %s\n', ...
        mat2str(priority_robots), demoScene);
    fprintf('========================================================\n');

    if ~exist('demoScene','var') || isempty(demoScene), demoScene = 'S1'; end
    const_base = const;   % snapshot — priority_n will be overwritten in loop

    panelPos_p = {[0.04 0.53 0.44 0.42],[0.52 0.53 0.44 0.42],...
                  [0.04 0.05 0.44 0.42],[0.52 0.05 0.44 0.42]};

    for prio_n = priority_robots
        fprintf('\n--- Priority Robot %d ---\n', prio_n);

        % Build const with priority override
        const_p           = const_base;
        const_p.priority_n = prio_n;

        % Run ADMM with priority constraint
        [xp, yp, LCp, rr_p, rs_p, dc_p, kp, Tp] = ...
            run_admm_core(const_p, agent_participation);

        % Folder and file names
        caseName_p = sprintf('%s_pR%d', caseName, prio_n);
        caseDir_p  = fullfile(rootSaveDir, caseName_p);
        if ~exist(caseDir_p, 'dir'), mkdir(caseDir_p); end

        % Save MAT — use standard variable names expected by export_scenario_to_demo
        const          = const_p;       % temporarily rebind for save
        x_prev         = xp;
        y_prev         = yp;
        LocalTreeCache  = LCp;
        residual_r     = rr_p;
        residual_s     = rs_p;
        delay_costs    = dc_p;
        k              = kp;
        T_ADMM_TOTAL   = Tp;

        matFile_p = fullfile(caseDir_p, sprintf('FourIntersection_ADMM_%s.mat', caseName_p));
        save(matFile_p, 'const', 'x_prev', 'y_prev', 'LocalTreeCache', ...
            'residual_r', 'residual_s', 'delay_costs', 'k', 'T_ADMM_TOTAL', ...
            'seed', 'Nveh');

        % Plot local schedule
        fig_p = figure('Color', 'w', 'Position', [60 40 1500 950]);
        for agent_i = 1:4
            cache = LCp{agent_i};
            if isempty(cache) || ~isstruct(cache), continue; end
            pp = uipanel('Parent', fig_p, 'Units', 'normalized', ...
                'Position', panelPos_p{agent_i}, ...
                'BackgroundColor', [0.97 0.97 0.97], 'BorderType', 'none');
            plot_local_schedule_final_into_panel_1(pp, cache.NODES, cache.Path, ...
                agent_i, cache.valid_systems, const_p, 'x_prev', xp, ...
                'gap', 0.004, 'panelColor', 'w', 'axColor', [0.98 0.98 0.98], ...
                'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0);
        end
        drawnow; pause(0.3);
        localBase_p = fullfile(caseDir_p, sprintf('local_%s', caseName_p));
        savefig(fig_p, [localBase_p '.fig']);
        print(fig_p, [localBase_p '.png'], '-dpng', '-r150');
        close(fig_p);

        % Export to HTML demo (priority_n passed → priorityRobot field in JS)
        demoScene_p = sprintf('%s_p%d', demoScene, prio_n);
        export_scenario_to_demo(caseName_p, demoGroup, demoScene_p, prio_n);

        fprintf('Priority R%d → scene %s done.\n', prio_n, demoScene_p);

        % Restore const_base for next iteration
        const = const_base;
    end

    fprintf('\n=== Priority Override Complete ===\n');
    fprintf('Add to HTML AVAILABLE map:\n');
    for prio_n = priority_robots
        demoScene_p = sprintf('%s_p%d', demoScene, prio_n);
        fprintf('  ''%s_%s_optimal'': true,\n', demoGroup, demoScene_p);
    end
    fprintf('Add scenes to SCENARIO_GROUPS[''%s''].scenes:\n', demoGroup);
    for prio_n = priority_robots
        fprintf('  ''%s_p%d''\n', demoScene, prio_n);
    end
end

fprintf('Finished case: %s\n', caseName);
fprintf('Saved to: %s\n', caseDir);
    end  % iS
end  % iN

%% ═══════════════════════ Local agent functions ═══════════════════════════

% ── run_admm_core ──────────────────────────────────────────────────────────
% Initialise and run one full ADMM sweep.  All routing/timing data is read
% from const.  priority_n = 0 → normal run; >0 → that robot has highest
% priority at every contended resource slot.
%
% Returns the converged (or max-iter) x_prev, y_prev, LocalTreeCache,
% residuals, converged iteration k, and elapsed time in minutes.
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

% ── Initialisation ──────────────────────────────────────────────────────
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

% ── ADMM main loop ──────────────────────────────────────────────────────
for k = 1 : max_iter
    k %#ok<NOPRT>
    t_iter = tic;
    x_last = x_prev; y_last = y_prev;

    %% Step 1: dual variable update
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

        vehUpd{n} = struct('ax',{ax_loc},'ay',{ay_loc},'xbar',{xbar_loc},'ybar',{ybar_loc});
    end

    for n = 1:N
        for ag = 1:9
            a_x_new{ag}{n}    = vehUpd{n}.ax{ag};
            a_y_new{ag}{n}    = vehUpd{n}.ay{ag};
            x_prev_bar{ag}{n} = vehUpd{n}.xbar{ag};
            if ag <= 8
                y_prev_bar{ag}{n} = vehUpd{n}.ybar{ag};
            end
        end
    end

    %% Step 2: local updates (parallel or sequential)
    meta = cell(1,9);
    if const.useParallel
        f = parallel.FevalFuture.empty(0,9);
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
        % Sequential — no parfeval, easy to debug with breakpoints
        for agent_i = 1:9
            if agent_i >= 1 && agent_i <= 4
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
                if isempty(valid_systems)
                    meta{agent_i} = agent_update_intersection_stub_single(agent_i);
                else
                    meta{agent_i} = agent_update_intersection_single(const, agent_i, entries, valid_systems, ...
                        x_prev, y_prev, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i}, ...
                        LocalTreeCache{agent_i}, k);
                end
            elseif agent_i >= 5 && agent_i <= 8
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

    %% Merge results
    r_local = 0;
    for agent_i = 1:9
        S_res = meta{agent_i};
        fprintf('Agent %d time=%.3f worker=%d\n', agent_i, S_res.elapsed, S_res.worker);
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
            otherwise
                error('Unknown meta.kind = %s', S_res.kind);
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

    r %#ok<NOPRT>
    s %#ok<NOPRT>
    residual_r(k) = r;
    residual_s(k) = s;
    a_x = a_x_new; a_y = a_y_new;

    % ── checkpoint: save after every iteration so Ctrl+C is recoverable ──
    if mod(k, 10) == 0
        save('admm_checkpoint.mat', ...
            'x_prev', 'y_prev', 'LocalTreeCache', ...
            'residual_r', 'residual_s', 'delay_costs', 'k');
    end

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
fprintf('ADMM elapsed %.3f mins  (priority_n=%d)\n', T_ADMM, const.priority_n);
end

% ── run_admm_core end ──────────────────────────────────────────────────────

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
