clear; clc;
Time_begin = tic;

T_val = 2;   % matches for_demo MAIN_Parallel_Compute.m
Dt = 3;      % matches for_demo MAIN_Parallel_Compute.m
routeDict = generateTrafficSystem();
IntSpaceDB = makeIntSpaceDB();

config = {  %simu1                      % 02052026
    struct('entrance', 1, 'exits', [7 3]),
    struct('entrance', 4, 'exits', [1 3]),
    struct('entrance', 6, 'exits', [1]),
    struct('entrance', 7, 'exits', [4]),
    struct('entrance', 5, 'exits', [3]),
    struct('entrance', 3, 'exits', [2 4]),
    struct('entrance', 2, 'exits', [5]),
}; 
% config = {
%     struct('entrance', 2, 'exits', [7]),
%     struct('entrance', 6, 'exits', [3]),
%     %struct('entrance', 8, 'exits', []),
%     struct('entrance', 4, 'exits', [6  5  5]),
%     struct('entrance', 1, 'exits', [3  8  8]),
%     struct('entrance', 5, 'exits', [4  1]),
%     struct('entrance', 3, 'exits', [1  8  2]),
%     struct('entrance', 7, 'exits', [6  4]),
% };

vehicleConfig = {};
vid = 0;
for g = 1:length(config)
    ent = config{g}.entrance;
    exits = config{g}.exits;
    for j = 1:length(exits)
        vid = vid + 1;
        vehicleConfig{vid} = struct( ...
            'entrance', ent, ...
            'exit', exits(j), ...
            'entryIndex', j);
    end
end
config = vehicleConfig;

N = length(config);

% ── Load alpha_tilde from the original ADMM optimal run ─────────────────────
% This ensures FCFS uses EXACTLY the same vehicle arrival times as the optimal,
% making the two schedules directly comparable.
ADMM_MAT = 'C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Distributed\Four_int_040926_for_demo\FourIntersection_ADMM.mat';
S_admm = load(ADMM_MAT, 'const');
alpha_tilde = S_admm.const.alpha_tilde;   % cell(N,1), same as optimal
fprintf('Loaded alpha_tilde from ADMM mat (N=%d vehicles).\n', numel(alpha_tilde));
% ─────────────────────────────────────────────────────────────────────────────

Veh = struct([]);

for n = 1:N
    ent = config{n}.entrance;
    ex  = config{n}.exit;

    route = routeDict(ent, ex);

    Veh(n).entrance   = ent;
    Veh(n).exit       = ex;
    Veh(n).entryIndex = config{n}.entryIndex;
    Veh(n).intSeq     = route.int;
    Veh(n).subDir     = route.subDir;
    Veh(n).routeId    = route.routeId;
    Veh(n).NI         = numel(route.int);

    Veh(n).alpha0 = alpha_tilde{n};
end

const = struct();
const.N = N;
const.Dt = Dt;
const.Veh = Veh;
const.IntSpaceDB = IntSpaceDB;
const.alpha_tilde = alpha_tilde;
const.Smax = 3;
const.numInt = 4;
const.spacePerInt = 5;
const.Mtot = const.numInt * const.spacePerInt;
const.BIG_M = 1000; %ddl 变成一个很大的数当新任务产生时

NI = zeros(1,N);
for n = 1:N
    NI(n) = Veh(n).NI;
end
const.NI = NI;
%% Root Node Initialization
Smax = const.Smax;
Mtot = const.Mtot;

d0  = inf(1,N);
r0  = zeros(Smax,N);
o0  = zeros(1,N);
ni0 = zeros(1,N);
tw0 = 0;

U_c0 = zeros(N,Mtot);
U0   = zeros(N,Mtot);

g0 = 0;
f0 = 0;

gamma0 = cell(1,N);
alpha0_node = cell(1,N);
x0 = cell(N,1);
speed0 = cell(1,N);
ra_reset0 = zeros(Smax,N);

for n = 1:N
    d0(n) = Veh(n).alpha0;   % 第一项任务的release time
    gamma0{n} = NaN(1, NI(n));
    alpha0_node{n} = NaN(1, NI(n));
    x0{n} = cell(1, NI(n));
    speed0{n} = 0;
end

NODES = {{1,d0,r0,o0,tw0,ni0,0,U_c0,U0,g0,gamma0,f0,speed0,ra_reset0,x0,alpha0_node}};
OPEN = 1;
LEAF = [];

step = 0;
max_expand_steps = 50000;   % debug safeguard
%% --------------------------main loop---------------------------------
while ~isempty(OPEN)
    step = step + 1;
    if step > max_expand_steps
        error('Exceeded max_expand_steps. Possible tree explosion or repeated states.');
    end
    % if isempty(OPEN)
    %     disp('OPEN is empty. Stop.');
    %     break; 
    % end
    % fprintf('OPEN candidate f-values:\n');
    % for kk = 1:numel(OPEN)
    %     idx = OPEN(kk);
    %     fprintf('  node %d: f = %.6f, g = %.6f, tw = %.6f\n', ...
    %         idx, NODES{idx}{12}, NODES{idx}{10}, NODES{idx}{5});
    % end 
 
    [min_f, minIndex] = f_min2(NODES, OPEN);
    c_node_index = minIndex;
    % if c_node_index == 20
    %     stop = 1;
    % end
    %fprintf('\n========== STEP %d, expand node %d ==========\n', step, c_node_index);
    [NODES, OPEN, LEAF] = expand_array_global2(NODES, OPEN, c_node_index, LEAF, const);

    fprintf('After expansion:\n');
    fprintf('  #nodes = %d\n', size(NODES,1));
    fprintf('  OPEN = '); disp(OPEN);
    fprintf('  LEAF = '); disp(LEAF);

    latest_idx = size(NODES,1);
    c_node = NODES{latest_idx};

    fprintf('Latest node %d summary:\n', latest_idx);
    fprintf('  tw = '); disp(c_node{5});
    fprintf('  ni = '); disp(c_node{6});
    fprintf('  d  = '); disp(c_node{2});
    fprintf('  r  = \n'); disp(c_node{3});
end

%%---------------record optimal path--------------------------
Number_of_Nodes = size(NODES,1);

% if you already know the best leaf index: 
% best_leaf_idx = 40;

% option A: manually specify leaf
% [Path_min, Optimal_g_cost, best_leaf_idx] = extract_optimal_path_from_nodes( ...
%     NODES, LEAF, 'best_leaf_idx', 40); 

% option B: automatically choose the best leaf by g cost
[Path_min, Optimal_g_cost, best_leaf_idx] = extract_optimal_path_from_nodes( ...
    NODES, LEAF, 'criterion', 'g_min');

elapsedTime = toc;
fprintf('Elapsed time: %.4f seconds\n', elapsedTime);

disp(['Number_of_Nodes = ', num2str(Number_of_Nodes)]);
disp(['best_leaf_idx   = ', num2str(best_leaf_idx)]);
disp('Path_min = ');
disp(Path_min);
disp(['Optimal_g_cost  = ', num2str(Optimal_g_cost)]);

save('nodes.mat', 'NODES', 'LEAF', 'Path_min', 'Optimal_g_cost', 'best_leaf_idx');

%%    plots
%% plots
DATA = build_centralized_schedule_data(NODES, Path_min, const);

fig = plot_local_schedule_panel_2x2_centralized(DATA, const);

% ── Export FCFS result for HTML demo ────────────────────────────────────────
DEMO_DIR   = 'C:\Users\robin\OneDrive\Documents\Github_file\Traffic_Demo\schedules';
const_fcfs = const;
out_path   = fullfile(DEMO_DIR, '10r_S1_fcfs.js');
export_demo_json_fcfs(const_fcfs, DATA, '10r · S1', out_path, 'fcfs');
% ─────────────────────────────────────────────────────────────────────────────