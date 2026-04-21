function [NODES, LEAF, c_node_index, timed_out, elapsedTime] = runAstar(cfg)
% runAstar  Core A* expansion loop.
%   Called by Main.m (single run) and runBatch.m (batch comparison).
%
%   cfg must already contain: N, M, S, Map, C, T, Ni, alpha_tilde,
%                              initial_position, d1, useWeakRule,
%                              pruneNodes, timeout_s, v_max,
%                              detect_range, W.

tic;
N = cfg.N;  Ni = cfg.Ni;  S = cfg.S;  M = cfg.M;
v_max_vec = cfg.v_max * ones(1, N);

%% 计算worst-case时间上界（串行执行所有任务）
gen_times = zeros(1, N);
for n = 1:N
    gen_times(n) = cfg.d1(n) + (cfg.detect_range/2 - cfg.W/2) / v_max_vec(n);
end
total_C = sum(arrayfun(@(n) sum(cfg.C(:,n)) * Ni(n), 1:N));
T_bound = max(gen_times) + total_C;
fprintf('  [T_bound] worst-case sequential deadline = %.2f s\n', T_bound);

%% Initial node
d = zeros(1, N);
for n = 1:N
    d(n) = cfg.d1(n) + (cfg.detect_range/2 - cfg.W/2) / v_max_vec(n);
end

r        = zeros(S, N);
o        = zeros(1, N);
V_c      = zeros(N, M);
V        = zeros(N, M);
gamma    = cell(1, N);
speed    = zeros(N, max(Ni));
ra_reset = zeros(S, N);
x        = cell(N, 1);
for n = 1:N,  x{n} = cell(1, Ni(n));  end

ni   = zeros(1, N);
tw   = 0;  g = 0;  f = 0;
LEAF = [];

NODES = {{1, d, r, o, tw, ni, 0, V_c, V, g, gamma, f, speed, ra_reset, x, zeros(N,N), zeros(1,N)}};
OPEN  = 1;
c_node_index = 1;

%% A* loop
% T_bound assumes tw increases monotonically. Displacement (WeakRule ON)
% can reset vehicles to earlier times, violating this assumption.
% Only apply T_bound when WeakRule is OFF.
apply_TBound = cfg.useTBound && ~cfg.useWeakRule;

max_nodes = 30000;  % hard safety cap — prevents single-call explosion

timed_out = false;
while any(ni <= Ni)
    [NODES, OPEN, LEAF] = expand_array(NODES, OPEN, c_node_index, cfg.useWeakRule, cfg, LEAF);

    if cfg.pruneNodes && length(OPEN) > 1
        OPEN = prune_nodes_by_ni(NODES, OPEN);
    end

    if cfg.timeout_s > 0 && toc > cfg.timeout_s
        fprintf('  [TIMEOUT] %.0fs — %d nodes, %d complete paths\n', ...
            cfg.timeout_s, size(NODES,1), numel(LEAF));
        timed_out = true;
        break;
    end

    if size(NODES, 1) >= max_nodes
        fprintf('  [MAX_NODES] %d nodes reached — %d complete paths\n', ...
            size(NODES,1), numel(LEAF));
        timed_out = true;
        break;
    end

    % T_bound pruning: only valid for WeakRule OFF (tw is monotone without displacement)
    if apply_TBound
        over = OPEN(arrayfun(@(idx) NODES{idx}{5} > T_bound, OPEN));
        if ~isempty(over)
            fprintf('  [PRUNE] %d nodes pruned (tw > %.2f)\n', numel(over), T_bound);
            OPEN = OPEN(~ismember(OPEN, over));
        end
    end

    % Branch-and-bound: 剪掉g已超过当前最优complete path的节点
    if cfg.useBnB
        cl_now = complete_leaves(NODES, LEAF, Ni);
        if ~isempty(cl_now)
            [best_g, ~] = f_min(NODES, cl_now);
            before = numel(OPEN);
            OPEN = OPEN(arrayfun(@(idx) NODES{idx}{10} <= best_g, OPEN));
            pruned_bb = before - numel(OPEN);
            if pruned_bb > 0
                fprintf('  [B&B] pruned %d nodes (g > %.4f)\n', pruned_bb, best_g);
            end
        end
    end

    if ~isempty(OPEN)
        [~, c_node_index] = f_min(NODES, OPEN);
        ni = NODES{c_node_index}{6};
    else
        [~, c_node_index] = f_min(NODES, complete_leaves(NODES, LEAF, Ni));
        break;
    end
end

%% After loop
if timed_out
    if ~isempty(LEAF)
        cl = complete_leaves(NODES, LEAF, Ni);
        if ~isempty(cl)
            [~, c_node_index] = f_min(NODES, cl);
        else
            [~, c_node_index] = f_min(NODES, LEAF);
        end
    else
        c_node_index = 0;
    end
elseif ~isempty(OPEN)
    % while-condition became false: c_node_index is still in OPEN, not yet a LEAF.
    % Expand it once so it is properly added to LEAF, then pick best complete leaf.
    [NODES, OPEN, LEAF] = expand_array(NODES, OPEN, c_node_index, cfg.useWeakRule, cfg, LEAF);
    cl = complete_leaves(NODES, LEAF, Ni);
    if ~isempty(cl)
        [~, c_node_index] = f_min(NODES, cl);
    elseif ~isempty(LEAF)
        [~, c_node_index] = f_min(NODES, LEAF);
    else
        c_node_index = 0;
    end
end

elapsedTime = toc;
end

function out = complete_leaves(NODES, LEAF, Ni)
% Return only leaves where gamma is fully populated (all tasks recorded).
    out = [];
    for ii = 1:numel(LEAF)
        idx = LEAF(ii);
        gam = NODES{idx}{11};
        ok = true;
        for n = 1:numel(Ni)
            if isempty(gam{n}) || numel(gam{n}) < Ni(n)
                ok = false;  break;
            end
        end
        if ok,  out(end+1) = idx;  end  %#ok<AGROW>
    end
end
