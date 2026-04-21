% Single-intersection centralized A* scheduler
% Simulates the LOCAL computation of one intersection agent (1 vehicle / route).
tic;
clear all;

% ===================== Algorithm switches ====================================
cfg.useWeakRule  = true;   % 初始值无影响 — 下方循环自动跑 ON 和 OFF 两遍

cfg.pruneNodes   = false; % true  → standard A*: prune branches with same ni,
                          %         keeps only cheapest → 1 optimal leaf
                          % false → no pruning: all branches survive → multiple
                          %         leaves visible in plotInteractiveTree

cfg.useBnB       = false; % true  → Branch-and-Bound: prune nodes with g > best complete path
                          %         → fewer leaves, faster, only optimal/near-optimal shown
                          % false → keep all valid paths → all scheduling results visible
 
cfg.useTBound    = true;  % true  → prune nodes with tw > serial worst-case deadline
                          %         Only applied to WeakRule OFF (displacement invalidates
                          %         tw monotonicity for WeakRule ON — see runAstar.m)
                          % false → no T_bound pruning

cfg.timeout_s    = 30;   % max run time (s); 0 = no limit.
                          % On timeout: output all branches found so far,
                          % use best complete leaf if any, skip if none.
 
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
% cfg.routeAssignment = [2 2 5 4 1];   % ← set N and routes here
% cfg.routeAssignment = [1 2 4 5]; %exp in ccta
% cfg.routeAssignment = [1 2 4 5 11 8]; 
% cfg.routeAssignment = [1 4 5]; 
 cfg.routeAssignment = [1 2 1 7]; 
% cfg.routeAssignment = [5 2 4 10]; 

% Initial arrival offset d1(n) for each system (length = N).
% All zeros → all systems arrive simultaneously (max contention).
cfg.d1 = zeros(1, numel(cfg.routeAssignment));
% cfg.d1 = [0, 0, 0, 0, 0, 0];   % N1在0s到，N2延迟3s，N3延迟6s
% cfg.d1 = [14.5, 15.2, 15.5, 15];  
cfg.d1 = [0, 0, 0, 0]; 

% ===================== Intersection config ===================================
intCfg   = makeIntersectionConfig(); 
cfg.N    = numel(cfg.routeAssignment);           % number of systems
cfg.M    = intCfg.M;                              % 5 spaces (fixed by geometry)
cfg.S    = intCfg.S;                                % 3 sub-tasks max
cfg.Map  = intCfg.Map(:, cfg.routeAssignment);   % one column per system (repeats allowed)
cfg.C    = intCfg.C(:,   cfg.routeAssignment);

% ===================== Tasks: 1 vehicle per route ============================
cfg.T_period = 20;                              % headway (s) — unused for single-vehicle runs
cfg.T        = repmat({{cfg.T_period}}, cfg.N, 1);
cfg.Ni       = ones(1, cfg.N);

% ===================== Derived initial conditions ============================
cfg = buildCfgInitials(cfg);

% ===================== Run BOTH WeakRule=ON and OFF ==========================
results_both = struct([]);
for useWeak = [true, false]
    cfg.useWeakRule = useWeak;
    ruleStr = 'ON'; if ~useWeak, ruleStr = 'OFF'; end
    fprintf('\n--- WeakRule %s ---\n', ruleStr);

    [NODES, LEAF, c_node_index, timed_out, elapsedTime] = runAstar(cfg);

    fprintf('useWeakRule : %d\n', useWeak);
    fprintf('Total nodes : %d\n', size(NODES,1));
    fprintf('Elapsed time: %.4f s\n', elapsedTime);

    if c_node_index > 0
        resultNodes = [];
        idx = c_node_index;
        while idx > 0,  resultNodes = [idx, resultNodes];  idx = NODES{idx}{7};  end
        fprintf('Optimal g-cost: %.4f\n', NODES{resultNodes(end)}{10});
        log_cost = NODES{c_node_index}{10};
    else
        fprintf('(No complete path)\n');
        log_cost = NaN;
    end

    logResultCSV(cfg, log_cost, numel(LEAF), size(NODES,1), elapsedTime, timed_out);

    ri = numel(results_both) + 1;
    results_both(ri).NODES      = NODES;
    results_both(ri).LEAF       = LEAF;
    results_both(ri).c_node_idx = c_node_index;
    results_both(ri).timed_out  = timed_out;
    results_both(ri).elapsed    = elapsedTime;
    results_both(ri).useWeak    = useWeak;
end

% Use ON result for MATLAB explorer (primary)
cfg.useWeakRule = true;
r_on  = results_both(1);   % WeakRule ON
r_off = results_both(2);   % WeakRule OFF

save('nodes.mat', 'NODES');   % saves last run (OFF)

% ===================== HTML interactive demo (both ON+OFF in one file) =======
has_on  = ~isempty(r_on.LEAF);
has_off = ~isempty(r_off.LEAF);
if has_on || has_off
    exportHTML(r_on.NODES,  r_on.LEAF,  cfg, [], ...
               r_off.NODES, r_off.LEAF, r_on.elapsed, r_off.elapsed);
else
    fprintf('(Skipping HTML export — no complete leaves in either run)\n');
end

% ===================== Interactive tree explorer =============================
% Always draw the tree (even without complete leaves) so we can see what happened.
cfg.useWeakRule = true;
plotInteractiveTree(r_on.NODES, r_on.LEAF, cfg, r_on.elapsed);
cfg.useWeakRule = false;
plotInteractiveTree(r_off.NODES, r_off.LEAF, cfg, r_off.elapsed);
