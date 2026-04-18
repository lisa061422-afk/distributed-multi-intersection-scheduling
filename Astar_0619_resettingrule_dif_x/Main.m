% Single-intersection centralized A* scheduler
% Simulates the LOCAL computation of one intersection agent (1 vehicle / route).
tic;
clear all;

% ===================== Algorithm switches ====================================
cfg.useWeakRule  = true;  % true  → apply resetting rule (weak rule)
                          % false → disable resetting rule (compare optimality)

cfg.pruneNodes   = false; % true  → standard A*: prune branches with same ni,
                          %         keeps only cheapest → 1 optimal leaf
                          % false → no pruning: all branches survive → multiple
                          %         leaves visible in plotInteractiveTree

% ===================== Physical parameters ===================================
cfg.v_max        = 20;    % vehicle speed (m/s, consistent with Map/C scale)
cfg.detect_range = 510;   % detection zone diameter (m)
cfg.W            = 30;    % merging zone width (m)

% ===================== Systems and route assignment ==========================
% cfg.routeAssignment(n) = which route type system n uses.
% Multiple systems can share the same route type (repetition allowed).
%
% Route numbering (see makeIntersectionConfig for geometry):
%   Arm 1: route 1 (left), route 2 (straight), route 3 (right)
%   Arm 2: route 4 (left), route 5 (straight), route 6 (right)
%   Arm 3: route 7 (left), route 8 (straight), route 9 (right)
%   Arm 4: route10 (left), route11 (straight), route12 (right)
%
% Examples:
%   [1 2 3]       — 3 systems, one per direction from arm 1
%   [1 1 1]       — 3 systems all left-turning from arm 1
%   [1 5 9 10]    — 4 systems on different arms/directions
%   [2 2 5 5 8 8] — 6 systems, 2 per straight-through arm
% cfg.routeAssignment = [1 4 11 8];   % ← set N and routes here
cfg.routeAssignment = [2 2 5 4 1];   % ← set N and routes here

% Initial arrival offset d1(n) for each system (length = N).
% All zeros → all systems arrive simultaneously (max contention).
cfg.d1 = zeros(1, numel(cfg.routeAssignment));
% cfg.d1 = [0, 3, 6];   % N1在0s到，N2延迟3s，N3延迟6s
cfg.d1 = [0.2, 0.5, 0.5, 0, 0]; 

% ===================== Intersection config ===================================
intCfg   = makeIntersectionConfig(); 
cfg.N    = numel(cfg.routeAssignment);           % number of systems
cfg.M    = intCfg.M;                             % 5 spaces (fixed by geometry)
cfg.S    = intCfg.S;                             % 3 sub-tasks max
cfg.Map  = intCfg.Map(:, cfg.routeAssignment);   % one column per system (repeats allowed)
cfg.C    = intCfg.C(:,   cfg.routeAssignment);

% ===================== Tasks: 1 vehicle per route ============================
cfg.T_period = 20;                              % headway (s) — unused for single-vehicle runs
cfg.T        = repmat({{cfg.T_period}}, cfg.N, 1);
cfg.Ni       = ones(1, cfg.N);

% ===================== Derived initial conditions ============================
v_max = cfg.v_max * ones(1, cfg.N);

alpha_tilde      = cell(1, cfg.N);
initial_position = zeros(cfg.N, max(cfg.Ni));

for n = 1:cfg.N
    alpha_tilde{n}(1)     = (cfg.detect_range/2 - cfg.W/2) / v_max(n) + cfg.d1(n);
    initial_position(n,1) = -(cfg.detect_range/2 - cfg.W/2 + cfg.d1(n)*v_max(n));
    for i = 2:cfg.Ni(n)
        alpha_tilde{n}(i)     = alpha_tilde{n}(i-1) + cfg.T{n}{i-1};
        initial_position(n,i) = initial_position(n,i-1) - cfg.T{n}{i-1}*v_max(n);
    end
end

cfg.alpha_tilde      = alpha_tilde;
cfg.initial_position = initial_position;

% ===================== Initial node ==========================================
d = zeros(1, cfg.N);
for n = 1:cfg.N
    d(n) = cfg.d1(n) + (cfg.detect_range/2 - cfg.W/2) / v_max(n);
end

r        = zeros(cfg.S, cfg.N);
o        = zeros(1, cfg.N);
V_c      = zeros(cfg.N, cfg.M);
V        = zeros(cfg.N, cfg.M);
gamma    = cell(1, cfg.N);
speed    = zeros(cfg.N, max(cfg.Ni));
ra_reset = zeros(cfg.S, cfg.N);
x        = cell(cfg.N, 1);
for n = 1:cfg.N
    x{n} = cell(1, cfg.Ni(n));
end

ni   = zeros(1, cfg.N);
tw   = 0;
g    = 0;
f    = 0;
LEAF = [];

NODES = {{1, d, r, o, tw, ni, 0, V_c, V, g, gamma, f, speed, ra_reset, x, zeros(1,cfg.M)}};
OPEN  = 1;
c_node_index = 1;

% ===================== A* main loop ==========================================
while any(ni <= cfg.Ni)
    [NODES, OPEN, LEAF] = expand_array(NODES, OPEN, c_node_index, cfg.useWeakRule, cfg, LEAF);

    if cfg.pruneNodes && length(OPEN) > 1
        OPEN = prune_nodes_by_ni(NODES, OPEN);
    end

    if ~isempty(OPEN)
        [~, c_node_index] = f_min(NODES, OPEN);
        ni = NODES{c_node_index}{6};
    else
        [~, c_node_index] = f_min(NODES, LEAF);
        break;
    end
end

% ===================== Results ===============================================
elapsedTime = toc;
fprintf('\n=== Run complete ===\n');
fprintf('useWeakRule : %d\n', cfg.useWeakRule);
fprintf('Routes (N)  : %d\n', cfg.N);
fprintf('Total nodes : %d\n', size(NODES, 1));
fprintf('Elapsed time: %.4f s\n', elapsedTime);

resultNodes = [];
idx = c_node_index;
while idx > 0
    resultNodes = [idx, resultNodes];
    idx = NODES{idx}{7};
end
fprintf('Optimal path:   '); disp(resultNodes);
fprintf('Optimal g-cost: %.4f\n', NODES{resultNodes(end)}{10});

save('nodes.mat', 'NODES');

% ===================== Interactive tree explorer =============================
% Shows decision tree (left) + resource allocation (right).
% Navigate: click a leaf node  OR  press ← → arrow keys.
% Selected path is highlighted red on the tree.
plotInteractiveTree(NODES, LEAF, cfg);
