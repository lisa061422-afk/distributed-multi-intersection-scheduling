clear; clc;
Time_begin = tic;

% ===== manually select a distributed case folder =====
startDir = 'C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Distributed\Four_int_031526_test_dif_num_vehicles\BatchRuns';
caseDir = uigetdir(startDir, 'Select a case folder');

if isequal(caseDir, 0)
    error('No case folder selected.');
end

caseConfigPath = fullfile(caseDir, 'case_config.mat');
if ~isfile(caseConfigPath)
    error('case_config.mat not found in:\n%s', caseDir);
end

S = load(caseConfigPath);

fprintf('Loaded case from:\n%s\n', caseDir);
fprintf('seed = %d, Nveh = %d\n', S.seed, S.Nveh);

T_val = S.T_val;          % loaded from case_config (matches distributed T_val)
Dt    = 100 / S.v_max(1); % derived from case_config v_max → always matches distributed Dt
routeDict = generateTrafficSystem();
IntSpaceDB = makeIntSpaceDB();

config = S.config;              % 这里已经是“单车 config”了
N = length(config);
v_max = S.v_max;
detect_range = S.detect_range;

W = 20;
d1 = zeros(1,N);

alpha_tilde = cell(N,1);
Veh = struct([]);

for n = 1:N
    ent = config{n}.entrance; 
    ex  = config{n}.exits; 

    route = routeDict(ent, ex);

    Veh(n).entrance   = ent;
    Veh(n).exit       = ex;
    Veh(n).entryIndex = config{n}.entryIndex;
    Veh(n).intSeq     = route.int;
    Veh(n).subDir     = route.subDir;
    Veh(n).routeId    = route.routeId;
    Veh(n).NI         = numel(route.int);

    base = (detect_range(n)/2 - W/2) / v_max(n) + d1(n);
    alpha_tilde{n} = base + (Veh(n).entryIndex - 1) * T_val;
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
%% 检查单个example config
if isfield(config{1}, 'exits')
    fprintf('Loaded vehicle-level config from case_config.mat\n');
else
    error('Loaded config is not vehicle-level.');
end
seed = S.seed;
Nveh = S.Nveh;
fprintf('Running FCFS for case: seed=%d, N=%d\n', seed, Nveh);
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
    if c_node_index == 20
        stop = 1;
    end
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

elapsedTime = toc(Time_begin);
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

% ── Save FCFS result to the batch folder (for export_to_demo_fcfs.m) ───────
const_fcfs   = const;   % rename so it doesn't clash with 'const' when loaded later
fcfsMatFile  = fullfile(caseDir, sprintf('FCFS_seed%d_N%d.mat', seed, Nveh));
save(fcfsMatFile, 'const_fcfs', 'DATA', 'seed', 'Nveh');
fprintf('FCFS result saved to:\n  %s\n', fcfsMatFile);
% ───────────────────────────────────────────────────────────────────────────

fig = plot_local_schedule_panel_2x2_centralized(DATA, const);
% for agent_i = 1:const.numInt
%     valid_systems = DATA.valid_systems{agent_i};
% 
%     fig = plot_local_tree_schedule_compact_centralized( ...
%         NODES, Path_min, DATA, const, agent_i, valid_systems);
% 
%     set(fig, 'Name', sprintf('Intersection %d', agent_i));
% end