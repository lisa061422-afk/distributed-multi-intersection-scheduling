%% MAIN_Parallel_5int.m
% 5-intersection traffic map ADMM scheduler.
% Topology:
%   I1(1) — I2(2) — I5(5)
%    |        |
%   I3(3) — I4(4)
%
% Agents:  I1-I5 = 1-5 (intersection)
%          Road I1-I2=6, I2-I5=7, I1-I3=8, I2-I4=9, I3-I4=10
%          Terminal = 11
%
% Ports:   1=I1-N  2=I1-W  3=I2-N  4=I3-W  5=I3-S
%          6=I4-S  7=I4-E  8=I5-N  9=I5-E
%
% DOES NOT affect MAIN_Parallel_Compute.m or any warehouse files.

clear all;

% ===================== Mode switch =====================
configMode = 'random';   % 'manual' | 'random'

% ===================== Physical parameters =====================
T_val           = 2.0;   % same-entrance headway (s)
T_ent           = 1;   % cross-entrance stagger (s)
Dt              = 3.0;   % road travel time between intersections (s)
v_max_phys      = 20.0;  % vehicle speed on road (m/s)
v_int           = 10.0;  % vehicle speed inside intersection (m/s)
W               = 20.0;  % merging zone / intersection width (m)
detect_range_val = 510.0; % detection range (m)

DEMO_DIR    = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';
rootSaveDir = 'BatchRuns';
demoScene   = 'S2';      % change per run

% ===================== Batch settings =====================
if strcmp(configMode, 'manual')
    numVehiclesList = [10];
    seedList        = [0];
else
    numVehiclesList = [20];
    seedList        = [41409];
end

% ===================== Run mode =====================
runMode     = 'normal';   % 'normal' | 'priority'
useParallel = true;
priority_robots = [18];    % which robots get fixed priority (used when runMode='priority')

skip_normal_run       = strcmp(runMode, 'priority');
enable_priority_sweep = strcmp(runMode, 'priority');

if ~exist(rootSaveDir, 'dir'), mkdir(rootSaveDir); end

% ===================== Parallel pool (10 agents → use as many workers as allowed) =====================
NUM_AGENTS = 11;   % 5 int + 5 road + 1 terminal
clusterMax  = parcluster('local').NumWorkers;
NUM_WORKERS = min(NUM_AGENTS - 1, clusterMax);  % parfeval queues if fewer workers than agents

if useParallel && license('test','Distrib_Computing_Toolbox')
    p = gcp('nocreate');
    if isempty(p) || p.NumWorkers ~= NUM_WORKERS
        if ~isempty(p), delete(p); end
        parpool('local', NUM_WORKERS);
    end
end

%% ── Main batch loop ────────────────────────────────────────────────────────
for iN = 1:numel(numVehiclesList)
for iS = 1:numel(seedList)

    close all;
    Time_begin = tic;

    Nveh = numVehiclesList(iN);
    seed = seedList(iS);

    if strcmp(configMode, 'manual')
        caseName = sprintf('5int_manual_%dr_%s', Nveh, demoScene);
    else
        caseName = sprintf('5int_seed%d_N%d_%s', seed, Nveh, demoScene);
    end

    caseDir = fullfile(rootSaveDir, caseName);
    if ~exist(caseDir, 'dir'), mkdir(caseDir); end

    fprintf('\n====================================================\n');
    fprintf('5-INT  case: %s  [%s]\n', caseName, configMode);
    fprintf('====================================================\n');

    %% ADMM parameters
    rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 200;
    tol_r = 0.01; tol_s = 0.01;
    randInitScale = 0;

    %% Local intersection DB (I1..I5, same M1-M5 structure)
    IntSpaceDB     = makeIntSpaceDB_5int();
    LocalTreeCache = cell(NUM_AGENTS, 1);

    %% Config generation
    if strcmp(configMode, 'manual')
        % ── default 5-int test config (10 vehicles) ───────────────────────
        % Ports: 1=I1-N 2=I1-W 3=I2-N 4=I3-W 5=I3-S 6=I4-S 7=I4-E 8=I5-N 9=I5-E
        config_raw = {
            struct('entrance', 1, 'exits', [9 7]),   % I1-N → I5-E, I4-E
            struct('entrance', 3, 'exits', [4 6]),   % I2-N → I3-W, I4-S
            struct('entrance', 8, 'exits', [5 2]),   % I5-N → I3-S, I1-W
            struct('entrance', 6, 'exits', [9 3]),   % I4-S → I5-E, I2-N
            struct('entrance', 4, 'exits', [8]),     % I3-W → I5-N
        };
        vehicleList = []; stats = struct();
    else
        % ── Random config over 9 ports ────────────────────────────────────
        [config_raw, vehicleList, stats] = generateBalancedTrafficConfig_5int(Nveh, ...
            'Seed', seed, ...
            'MaxPerEntrance',     ceil(Nveh/4), ...
            'MaxPerIntersection', ceil(Nveh*1.5/5) + 2, ...
            'MaxPerStage',        ceil(Nveh/5) + 1, ...
            'EntrancePenalty',    0.4);
        configFile = fullfile(caseDir, sprintf('config_%s.m', caseName));
        printTrafficConfig(config_raw, configFile);
    end

    %% Expand route-based config → vehicle-based config
    vehicleConfig = {};
    vid = 0;
    for g = 1:length(config_raw)
        ent   = config_raw{g}.entrance;
        exits = config_raw{g}.exits;
        for j = 1:length(exits)
            vid = vid + 1;
            vehicleConfig{vid} = struct('entrance', ent, 'exits', exits(j), 'entryIndex', j);
        end
    end
    config = vehicleConfig;
    Nveh   = length(config);

    %% Routing (5-int versions)
    N        = length(config);
    pathInfo = getVehiclePaths_5int(config);

    pathInfo_agent_chain = cell(1, N);
    pathInfo_c           = cell(1, N);
    TERMINAL             = NUM_AGENTS;   % = 11

    for n = 1:N
        kn = 1;
        pathInfo_agent_chain{n} = cell(1, kn);
        pathInfo_c{n}           = cell(1, 1);
        int_seq  = pathInfo{n}(kn).int;
        routeIds = pathInfo{n}(kn).routeId;

        dur = zeros(1, numel(int_seq));
        for k_int = 1:numel(int_seq)
            ag  = int_seq(k_int);
            rId = routeIds(k_int);
            dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});
        end

        ag_chain = [];
        for ii = 1:length(int_seq)-1
            road_ag  = getRoadAgent_5int(int_seq(ii), int_seq(ii+1));
            ag_chain = [ag_chain, int_seq(ii), road_ag];
        end
        ag_chain = [ag_chain, int_seq(end), TERMINAL];
        pathInfo_agent_chain{n}{kn} = ag_chain;
        pathInfo_c{n}{kn}           = dur;
    end

    %% agent_participation  (agents 1-10; terminal=11 handled separately)
    NUM_NON_TERMINAL = NUM_AGENTS - 1;   % = 10
    agent_participation = repmat({cell(N, 1)}, NUM_NON_TERMINAL, 1);
    for n = 1:N
        kn    = 1;
        chain = pathInfo_agent_chain{n}{kn};
        ags   = chain(1:end-1);   % exclude terminal
        for ii = 1:numel(ags)
            ag = ags(ii);
            agent_participation{ag}{n} = 1;
        end
    end

    %% Physical setup
    v_max        = v_max_phys * ones(1, N);
    d1           = zeros(1, N);
    detect_range = detect_range_val * ones(1, N);

    all_entrances = cellfun(@(c) c.entrance, config);
    unique_ents   = unique(all_entrances, 'stable');
    ent_rank      = zeros(1, max(unique_ents));
    for ei = 1:numel(unique_ents)
        ent_rank(unique_ents(ei)) = ei - 1;
    end

    alpha_tilde      = cell(N, 1);
    initial_position = zeros(N, 1);
    for n = 1:N
        alpha_tilde{n} = zeros(1,1);
        base = (detect_range(n)/2 - W/2) / v_max(n) + d1(n);
        alpha_tilde{n}(1) = base ...
            + ent_rank(config{n}.entrance) * T_ent ...
            + (config{n}.entryIndex - 1)   * T_val;
        t_arr = alpha_tilde{n}(1);
        initial_position(n) = -(detect_range(n)/2 - W/2 + t_arr * v_max(n));
    end

    speed = cell(N,1);
    for n = 1:N, speed{n} = 0; end

    %% Deadlines
    deadline = cell(N, 1);
    for n = 1:N
        kn = 1;
        chain     = pathInfo_agent_chain{n}{kn};
        num_roads = floor((length(chain) - 1) / 2);
        c_total   = sum(pathInfo_c{n}{kn});
        deadline{n}    = zeros(kn, 1);
        deadline{n}(kn) = alpha_tilde{n}(kn) + c_total + Dt * num_roads;
    end

    %% Pack const
    const = struct();
    const.rho1   = rho1;   const.rho2   = rho2;
    const.weight = weight; const.max_iter = max_iter;
    const.tol_r  = tol_r;  const.tol_s   = tol_s;
    const.N      = N;
    const.Dt     = Dt;
    const.priority_n       = 0;
    const.use_pruning      = true;
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
    const.useParallel      = useParallel;
    const.NUM_AGENTS       = NUM_AGENTS;
    % Physical parameters (for reproducibility)
    const.T_val            = T_val;
    const.T_ent            = T_ent;
    const.v_max_phys       = v_max_phys;
    const.v_int            = v_int;
    const.W                = W;
    const.detect_range_val = detect_range_val;
    const.configMode       = configMode;
    const.demoScene        = demoScene;

    demoGroup = sprintf('%dr', Nveh);
    matFile   = fullfile(caseDir, sprintf('5int_ADMM_%s.mat', caseName));

    if skip_normal_run
        load(matFile, 'x_prev','y_prev','LocalTreeCache','residual_r','residual_s','delay_costs','k');
    else
        %% ── ADMM run ─────────────────────────────────────────────────────
        [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, delay_costs, k, T_ADMM] = ...
            run_admm_5int(const, agent_participation, NUM_AGENTS);
        fprintf('ADMM elapsed %.3f mins\n', T_ADMM);

        save(matFile, 'config','vehicleList','stats','const', ...
            'residual_r','residual_s','x_prev','y_prev','max_iter','k', ...
            'delay_costs','LocalTreeCache','T_ADMM','seed','Nveh');
        load(matFile);

        %% ── Plot macro schedule ──────────────────────────────────────────
        figs_before = findall(0,'Type','figure');
        plot_C_ADMM2_5int(residual_r, residual_s, delay_costs, ...
            x_prev, y_prev, const.pathInfo_agent_chain, N, k);
        drawnow;
        figs_after = findall(0,'Type','figure');
        new_figs   = setdiff(figs_after, figs_before);
        [~, ord]   = sort(arrayfun(@(f) f.Number, new_figs));
        new_figs   = new_figs(ord);
        fig_macro  = new_figs(min(3, end));

        macroBase = fullfile(caseDir, sprintf('macro_%s', caseName));
        savefig(fig_macro, [macroBase '.fig']);
        exportgraphics(fig_macro, [macroBase '.png'], 'Resolution', 300);

        %% ── Plot local schedules (2×3 grid for 5 intersections) ──────────
        fig = figure('Color','w','Position',[50 50 2200 1080]);
        panelPos = {
            [0.02  0.53  0.30  0.42],   % I1
            [0.35  0.53  0.30  0.42],   % I2
            [0.68  0.53  0.30  0.42],   % I5
            [0.02  0.05  0.30  0.42],   % I3
            [0.35  0.05  0.30  0.42]};  % I4
        int_order = [1 2 5 3 4];        % display order matches topology

        for pi = 1:5
            agent_i = int_order(pi);
            cache   = LocalTreeCache{agent_i};
            if isempty(cache) || ~isstruct(cache), continue; end
            p = uipanel('Parent', fig, 'Units','normalized', ...
                'Position', panelPos{pi}, ...
                'BackgroundColor',[0.97 0.97 0.97], 'BorderType','none');
            plot_local_schedule_final_into_panel_1(p, ...
                cache.NODES, cache.Path, agent_i, cache.valid_systems, const, ...
                'x_prev', x_prev, ...
                'gap',0.004,'panelColor','w','axColor',[0.98 0.98 0.98], ...
                'marg_w',0.12,'marg_h',0.08,'title_pad',0);
        end
        drawnow; pause(0.3);
        localBase = fullfile(caseDir, sprintf('local_%s', caseName));
        savefig(fig, [localBase '.fig']);
        exportapp(fig, [localBase '.png']);

        %% ── FCFS (centralized BnB) ───────────────────────────────────────
        fprintf('\n===== Running FCFS =====\n');
        NUM_INT_FCFS = 5;

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
        const_fcfs.numInt     = NUM_INT_FCFS;
        const_fcfs.spacePerInt = 5;
        const_fcfs.Mtot       = NUM_INT_FCFS * 5;   % 25
        const_fcfs.BIG_M      = 1000;
        const_fcfs.NI         = NI_fcfs;

        Smax_f  = 3; Mtot_f = const_fcfs.Mtot;
        d0_f    = inf(1,N);   r0_f = zeros(Smax_f,N);
        o0_f    = zeros(1,N); ni0_f = zeros(1,N);
        U_c0_f  = zeros(N,Mtot_f); U0_f = zeros(N,Mtot_f);
        gamma0_f = cell(1,N); alpha0_f = cell(1,N);
        x0_f = cell(N,1); speed0_f = cell(1,N);
        ra_reset0_f = zeros(Smax_f,N);
        for n = 1:N
            d0_f(n) = Veh_fcfs(n).alpha0;
            gamma0_f{n} = NaN(1, NI_fcfs(n));
            alpha0_f{n} = NaN(1, NI_fcfs(n));
            x0_f{n}     = cell(1, NI_fcfs(n));
            speed0_f{n} = 0;
        end

        NODES_fcfs = {{1,d0_f,r0_f,o0_f,0,ni0_f,0,U_c0_f,U0_f,0, ...
                       gamma0_f,0,speed0_f,ra_reset0_f,x0_f,alpha0_f}};
        OPEN_fcfs = 1; LEAF_fcfs = []; step_fcfs = 0;
        Time_fcfs = tic;
        while ~isempty(OPEN_fcfs)
            step_fcfs = step_fcfs + 1;
            if step_fcfs > 100000
                warning('FCFS: exceeded max steps'); break;
            end
            [~, midx] = f_min2(NODES_fcfs, OPEN_fcfs);
            [NODES_fcfs, OPEN_fcfs, LEAF_fcfs] = expand_array_global2( ...
                NODES_fcfs, OPEN_fcfs, midx, LEAF_fcfs, const_fcfs);
        end
        fprintf('FCFS: %.2f s  (%d steps)\n', toc(Time_fcfs), step_fcfs);

        [Path_fcfs, Cost_fcfs] = extract_optimal_path_from_nodes( ...
            NODES_fcfs, LEAF_fcfs, 'criterion','g_min');
        DATA_fcfs = build_centralized_schedule_data(NODES_fcfs, Path_fcfs, const_fcfs);

        fcfsMatFile = fullfile(caseDir, sprintf('FCFS_%s.mat', caseName));
        save(fcfsMatFile, 'const_fcfs','DATA_fcfs','seed','Nveh','Cost_fcfs');

        fig_fcfs = figure('Color','w','Position',[50 50 2200 1080]);
        for pi = 1:5
            agent_i = int_order(pi);
            vs = DATA_fcfs.valid_systems{agent_i};
            if isempty(vs), continue; end
            pf = uipanel('Parent',fig_fcfs,'Units','normalized', ...
                'Position',panelPos{pi}, ...
                'BackgroundColor',[0.97 0.97 0.97],'BorderType','none');
            plot_local_schedule_final_into_panel_centralized(pf, DATA_fcfs, const_fcfs, ...
                agent_i, vs, ...
                'gap',0.004,'panelColor','w','axColor',[0.98 0.98 0.98], ...
                'marg_w',0.12,'marg_h',0.08,'title_pad',0);
        end
        drawnow; pause(0.3);
        fcfsBase = fullfile(caseDir, sprintf('fcfs_%s', caseName));
        savefig(fig_fcfs, [fcfsBase '.fig']);
        exportapp(fig_fcfs, [fcfsBase '.png']);

        fprintf('Finished: %s\n', caseDir);
    end  % ~skip_normal_run

    %% ── Priority Override (5-int) ────────────────────────────────────────
    if enable_priority_sweep && ~isempty(priority_robots)
        fprintf('\n========================================================\n');
        fprintf('Priority Override: robots %s, scene base = %s\n', ...
            mat2str(priority_robots), demoScene);
        fprintf('========================================================\n');

        % Load normal run result as baseline
        matFile_base = fullfile(caseDir, sprintf('5int_ADMM_%s.mat', caseName));
        if ~exist(matFile_base, 'file')
            error('Normal run MAT not found: %s', matFile_base);
        end
        base = load(matFile_base);
        const_base = base.const;

        panelPos_p = {
            [0.02 0.53 0.30 0.42], [0.35 0.53 0.30 0.42], [0.68 0.53 0.30 0.42], ...
            [0.02 0.05 0.30 0.42], [0.35 0.05 0.30 0.42]};
        int_order = [1 2 5 3 4];

        for prio_n = priority_robots
            fprintf('\n--- Priority Robot %d ---\n', prio_n);

            const_p = const_base;
            const_p.priority_n = prio_n;

            [xp, yp, LCp, rr_p, rs_p, dc_p, kp, Tp] = ...
                run_admm_5int(const_p, const_p.agent_participation, NUM_AGENTS);

            % Folder
            caseName_p = sprintf('%s_pR%d', caseName, prio_n);
            caseDir_p  = fullfile(rootSaveDir, caseName_p);
            if ~exist(caseDir_p, 'dir'), mkdir(caseDir_p); end

            % Save
            const          = const_p;
            x_prev         = xp;
            y_prev         = yp;
            LocalTreeCache = LCp;
            residual_r     = rr_p;
            residual_s     = rs_p;
            delay_costs    = dc_p;
            k              = kp;
            T_ADMM         = Tp;

            matFile_p = fullfile(caseDir_p, sprintf('5int_ADMM_%s.mat', caseName_p));
            save(matFile_p, 'const', 'x_prev', 'y_prev', 'LocalTreeCache', ...
                'residual_r', 'residual_s', 'delay_costs', 'k', 'T_ADMM', ...
                'seed', 'Nveh');

            % Plot macro
            figs_before = findall(0,'Type','figure');
            plot_C_ADMM2_5int(residual_r, residual_s, delay_costs, ...
                x_prev, y_prev, const_p.pathInfo_agent_chain, Nveh, k);
            drawnow;
            figs_after = findall(0,'Type','figure');
            new_figs   = setdiff(figs_after, figs_before);
            [~, ord2]  = sort(arrayfun(@(f) f.Number, new_figs));
            new_figs   = new_figs(ord2);
            fig_macro_p = new_figs(min(3, end));
            macroBase_p = fullfile(caseDir_p, sprintf('macro_%s', caseName_p));
            savefig(fig_macro_p, [macroBase_p '.fig']);
            exportgraphics(fig_macro_p, [macroBase_p '.png'], 'Resolution', 300);

            % Plot local schedule (2x3 grid)
            fig_p = figure('Color','w','Position',[50 50 2200 1080]);
            for pi = 1:5
                agent_i = int_order(pi);
                cache = LCp{agent_i};
                if isempty(cache) || ~isstruct(cache), continue; end
                p = uipanel('Parent', fig_p, 'Units','normalized', ...
                    'Position', panelPos_p{pi}, ...
                    'BackgroundColor',[0.97 0.97 0.97], 'BorderType','none');
                plot_local_schedule_final_into_panel_1(p, ...
                    cache.NODES, cache.Path, agent_i, cache.valid_systems, const_p, ...
                    'x_prev', xp, ...
                    'gap',0.004,'panelColor','w','axColor',[0.98 0.98 0.98], ...
                    'marg_w',0.12,'marg_h',0.08,'title_pad',0);
            end
            drawnow; pause(0.3);
            localBase_p = fullfile(caseDir_p, sprintf('local_%s', caseName_p));
            savefig(fig_p, [localBase_p '.fig']);
            exportapp(fig_p, [localBase_p '.png']);

            % Export to demo
            demoScene_p = sprintf('%s_p%d', demoScene, prio_n);
            export_scenario_to_demo_5int(caseName_p, demoGroup, demoScene_p);

            fprintf('Priority R%d done.\n', prio_n);
            const = const_base;
        end

        fprintf('\n=== Priority Override Complete ===\n');
        fprintf('Add to HTML AVAILABLE_TR:\n');
        for prio_n = priority_robots
            fprintf('  ''%s_%s_p%d_optimal'': true,\n', demoGroup, demoScene, prio_n);
        end
    end

end  % iS
end  % iN


%% ═══════════════════════ Local functions ════════════════════════════════════

function [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, ...
          delay_costs, k, T_ADMM] = run_admm_5int(const, agent_participation, NUM_AGENTS)

N        = const.N;
Dt       = const.Dt;
max_iter = const.max_iter;
tol_r    = const.tol_r;
tol_s    = const.tol_s;
rho1     = const.rho1;
alpha_tilde          = const.alpha_tilde;
pathInfo_agent_chain = const.pathInfo_agent_chain;
pathInfo_c           = const.pathInfo_c;
TERMINAL             = NUM_AGENTS;          % = 11
NUM_NON_TERMINAL     = NUM_AGENTS - 1;      % = 10
NUM_INT              = 5;
NUM_ROAD_FIRST       = NUM_INT + 1;         % = 6  (first road agent index)
NUM_ROAD_LAST        = NUM_NON_TERMINAL;    % = 10 (last  road agent index)

LocalTreeCache = cell(NUM_AGENTS, 1);

[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain_5int(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N, const.randInitScale);

% Dual variables
a_x     = cell(1, NUM_AGENTS);
a_y     = cell(1, NUM_AGENTS);
a_x_new = cell(1, NUM_AGENTS);
a_y_new = cell(1, NUM_AGENTS);
for i = 1:NUM_AGENTS
    a_x{i}     = cell(1,N); a_y{i}     = cell(1,N);
    a_x_new{i} = cell(1,N); a_y_new{i} = cell(1,N);
    [a_x{i}{:}]     = deal(0); [a_y{i}{:}]     = deal(0);
    [a_x_new{i}{:}] = deal(0); [a_y_new{i}{:}] = deal(0);
end

residual_r  = zeros(max_iter,1);
residual_s  = zeros(max_iter,1);
delay_costs = zeros(max_iter,1);
T0 = tic;

for k = 1:max_iter
    k %#ok<NOPRT>
    t_iter = tic;
    x_last = x_prev; y_last = y_prev;

    %% Step 1: dual variable update
    vehUpd = cell(N,1);
    for n = 1:N
        kn    = 1;
        chain = pathInfo_agent_chain{n}{kn};

        ax_loc   = cell(1, NUM_AGENTS);
        ay_loc   = cell(1, NUM_AGENTS);
        xbar_loc = cell(1, NUM_AGENTS);
        ybar_loc = cell(1, NUM_NON_TERMINAL);

        for ag = 1:NUM_AGENTS
            ax_loc{ag}   = a_x{ag}{n};
            ay_loc{ag}   = a_y{ag}{n};
            xbar_loc{ag} = x_prev_bar{ag}{n};
            if ag <= NUM_NON_TERMINAL
                ybar_loc{ag} = y_prev_bar{ag}{n};
            end
        end

        ag0 = chain(1);
        xbar_loc{ag0}(kn) = (x_prev{ag0}{n}(kn) + 0) / 2;

        for pos = 2:length(chain)
            prev_ag = chain(pos-1);
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
        for ag = 1:NUM_AGENTS
            a_x_new{ag}{n}    = vehUpd{n}.ax{ag};
            a_y_new{ag}{n}    = vehUpd{n}.ay{ag};
            x_prev_bar{ag}{n} = vehUpd{n}.xbar{ag};
            if ag <= NUM_NON_TERMINAL
                y_prev_bar{ag}{n} = vehUpd{n}.ybar{ag};
            end
        end
    end

    %% Step 2: local updates
    meta = cell(1, NUM_AGENTS);
    if const.useParallel
        f = parallel.FevalFuture.empty(0, NUM_AGENTS);
        % Submit in priority order: intersections first (heavy BnB),
        % then roads, then terminal — so intersections grab workers first.
        submit_order = [1:NUM_INT, NUM_ROAD_FIRST:NUM_ROAD_LAST, TERMINAL];
        for agent_i = submit_order
            entries       = [];
            valid_systems = [];
            if agent_i <= NUM_NON_TERMINAL
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
            end

            if agent_i >= 1 && agent_i <= NUM_INT          % intersection
                if isempty(valid_systems)
                    f(agent_i) = parfeval(@stub_int, 1, agent_i);
                else
                    f(agent_i) = parfeval(@upd_int, 1, ...
                        const, agent_i, entries, valid_systems, ...
                        x_prev, y_prev, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i}, ...
                        {}, k);  % pass empty cache: INi_Admm_DecisionTree rebuilds NODES each iter
                end
            elseif agent_i >= NUM_ROAD_FIRST && agent_i <= NUM_ROAD_LAST  % road
                if isempty(valid_systems)
                    f(agent_i) = parfeval(@stub_road, 1, agent_i);
                else
                    f(agent_i) = parfeval(@upd_road, 1, ...
                        const, agent_i, entries, valid_systems, ...
                        x_prev{agent_i}, y_prev{agent_i}, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i});
                end
            else                                            % terminal (11)
                f(agent_i) = parfeval(@upd_terminal, 1, ...
                    const, x_prev{TERMINAL}, x_prev_bar{TERMINAL}, a_x_new{TERMINAL});
            end
        end
        for ii = 1:NUM_AGENTS
            try
                [cidx, S_res] = fetchNext(f);
                meta{cidx} = S_res;
            catch ME
                fprintf(2,'[Iter %d] fetchNext error:\n%s\n', k, ...
                    getReport(ME,'extended','hyperlinks','off'));
                rethrow(ME);
            end
        end
    else
        % Sequential (debug mode)
        for agent_i = 1:NUM_AGENTS
            entries = []; valid_systems = [];
            if agent_i <= NUM_NON_TERMINAL
                entries       = agent_participation{agent_i};
                valid_systems = find(~cellfun(@isempty, entries))';
            end
            if agent_i >= 1 && agent_i <= NUM_INT
                if isempty(valid_systems)
                    meta{agent_i} = stub_int(agent_i);
                else
                    meta{agent_i} = upd_int(const, agent_i, entries, valid_systems, ...
                        x_prev, y_prev, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i}, ...
                        LocalTreeCache{agent_i}, k);
                end
            elseif agent_i >= NUM_ROAD_FIRST && agent_i <= NUM_ROAD_LAST
                if isempty(valid_systems)
                    meta{agent_i} = stub_road(agent_i);
                else
                    meta{agent_i} = upd_road(const, agent_i, entries, valid_systems, ...
                        x_prev{agent_i}, y_prev{agent_i}, ...
                        x_prev_bar{agent_i}, y_prev_bar{agent_i}, ...
                        a_x_new{agent_i}, a_y_new{agent_i});
                end
            else
                meta{agent_i} = upd_terminal(const, x_prev{TERMINAL}, x_prev_bar{TERMINAL}, a_x_new{TERMINAL});
            end
        end
    end

    %% Merge
    r_local = 0;
    for agent_i = 1:NUM_AGENTS
        S = meta{agent_i};
        if isempty(S), continue; end
        fprintf('Agent %d  t=%.3fs\n', agent_i, S.elapsed);
        switch S.kind
            case 'intersection'
                LocalTreeCache{agent_i} = S.cache;
                for n = S.valid_systems
                    kn = 1;
                    x_prev{agent_i}{n}(kn) = S.best_x(n);
                    y_prev{agent_i}{n}(kn) = S.best_y(n);
                    r_local = r_local + ...
                        (S.best_x(n) - S.best_alpha{n}(kn))^2 + ...
                        (S.best_y(n) - S.best_gamma{n}(kn))^2;
                end
            case 'road'
                for n = S.valid_systems
                    x_prev{agent_i}{n}(1) = S.x_road(n);
                    y_prev{agent_i}{n}(1) = S.y_road(n);
                end
            case 'terminal'
                delay_costs(k) = S.delay_cost;
                for n = 1:N
                    x_prev{TERMINAL}{n}(1) = S.x9_new(n);
                end
        end
    end

    %% Residuals
    r = compute_r(x_prev, y_prev, r_local, const);
    s = 0;
    for agent_i = 1:NUM_NON_TERMINAL
        ent = agent_participation{agent_i};
        if all(cellfun(@isempty, ent)), continue; end
        vs = find(~cellfun(@isempty, ent))';
        for n = vs
            s = s + norm(x_prev{agent_i}{n} - x_last{agent_i}{n})^2 ...
                  + norm(y_prev{agent_i}{n} - y_last{agent_i}{n})^2;
        end
    end
    for n = 1:N
        s = s + norm(x_prev{TERMINAL}{n} - x_last{TERMINAL}{n})^2;
    end

    residual_r(k) = r; residual_s(k) = s;
    r, s %#ok<NOPRT>
    a_x = a_x_new; a_y = a_y_new;

    if mod(k,10)==0
        save('admm_checkpoint_5int.mat','x_prev','y_prev','LocalTreeCache', ...
            'residual_r','residual_s','delay_costs','k');
    end

    if r < tol_r && s < tol_s
        fprintf('Converged at iteration %d\n', k);
        residual_r  = residual_r(1:k);
        residual_s  = residual_s(1:k);
        delay_costs = delay_costs(1:k);
        break;
    end
    fprintf('[Iter %d] %.3f s\n', k, toc(t_iter));
end

T_ADMM = toc(T0)/60;
fprintf('ADMM elapsed %.3f mins\n', T_ADMM);
end


%% ── Agent update helpers ─────────────────────────────────────────────────

function S = upd_int(const, agent_i, entries, valid_systems, ...
        x_prev_all, y_prev_all, xi_bar, yi_bar, ai_x, ai_y, cache_ai, k)
    t0 = tic;
    task = getCurrentTask(); wid = -1;
    if ~isempty(task), wid = task.ID; end
    if isempty(cache_ai) || ~isstruct(cache_ai), cache_ai = struct(); end
    [bx, by, ba, bg, bidx, ~, cache_ai] = INi_Admm_DecisionTree( ...
        agent_i, entries, x_prev_all, y_prev_all, xi_bar, yi_bar, ...
        valid_systems, ai_x, ai_y, const, cache_ai);
    if isempty(cache_ai) || ~isstruct(cache_ai), cache_ai = struct(); end
    cache_ai.iter = k;
    S = struct('kind','intersection','agent',agent_i,'valid_systems',valid_systems, ...
        'best_x',bx,'best_y',by,'best_alpha',{ba},'best_gamma',{bg}, ...
        'best_idx',bidx,'cache',cache_ai,'elapsed',toc(t0),'worker',wid);
end

function S = upd_road(const, agent_i, entries, valid_systems, ...
        x_i, y_i, xi_bar, yi_bar, ai_x, ai_y)
    t0 = tic;
    task = getCurrentTask(); wid = -1;
    if ~isempty(task), wid = task.ID; end
    [xr, yr] = updateRoadAgent(agent_i, entries, valid_systems, ...
        x_i, y_i, xi_bar, yi_bar, ai_x, ai_y, const);
    S = struct('kind','road','agent',agent_i,'valid_systems',valid_systems, ...
        'x_road',xr,'y_road',yr,'elapsed',toc(t0),'worker',wid);
end

function S = upd_terminal(const, x9, xbar9, ax9)
    t0 = tic;
    task = getCurrentTask(); wid = -1;
    if ~isempty(task), wid = task.ID; end
    [x9n, dc] = updateAgent9(x9, xbar9, ax9, const);
    % Field-by-field: prevents MATLAB struct()-expansion when x9n is a cell/vector
    S = struct();
    S.kind      = 'terminal';
    S.x9_new    = x9n;
    S.delay_cost = dc;
    S.elapsed   = toc(t0);
    S.worker    = wid;
end

function S = stub_int(agent_i)
    task = getCurrentTask(); wid = -1;
    if ~isempty(task), wid = task.ID; end
    S = struct('kind','intersection','agent',agent_i,'valid_systems',[], ...
        'best_x',[],'best_y',[],'best_alpha',{{}},'best_gamma',{{}}, ...
        'best_idx',[],'cache',[],'elapsed',0,'worker',wid);
end

function S = stub_road(agent_i)
    task = getCurrentTask(); wid = -1;
    if ~isempty(task), wid = task.ID; end
    S = struct('kind','road','agent',agent_i,'valid_systems',[], ...
        'x_road',[],'y_road',[],'elapsed',0,'worker',wid);
end
