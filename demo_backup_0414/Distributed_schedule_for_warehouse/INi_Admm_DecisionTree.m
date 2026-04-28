function [x, y, best_alpha, best_gamma, best_idx, NODES,LocalTreeCache] = ...
            INi_Admm_DecisionTree(agent_i, entries,...
                x_prev, y_prev, xi_prev_bar, yi_prev_bar,...
                valid_systems, ai_x, ai_y, const,LocalTreeCache)

if isempty(LocalTreeCache) || ~isstruct(LocalTreeCache)
    LocalTreeCache = struct();
end

%global M NI_agent % Dt pathInfo pathInfo_agent_chain S alpha_tilde
   
IntSpaceDB = const.IntSpaceDB;
N = const.N; Dt = const.Dt; alpha_tilde = const.alpha_tilde;
pathInfo_agent_chain = const.pathInfo_agent_chain;
pathInfo = const.pathInfo; pathInfo_c = const.pathInfo_c;

% 找到所有经过该 agent_i 的 system 索引
NI_agent = NaN(1, N);  % 初始化为 NaN
NI_agent(valid_systems) = 1;
M = IntSpaceDB{agent_i}.numSpaces; 
S = 3;   
d = NaN(1, N); % 取决于该intersection agent 在序列的哪个位置
ddl = cell(1, N);        % 每个 system n 存所有参与的 kn 的 ddl
T_est = cell(1, N);      % 每个 system n 存对应任务的周期估计
pos_map = cell(1, N);    % 每个 system n 存所有 kn 的位置
kn = 1; %现在每个系统只有一辆车
T_cst = 16.111;
Cmat = zeros(S, N); MapMat = zeros(S, N); 

arrival_ref = NaN(1, N);
for n = valid_systems
    % if isempty(entries{n}), continue; end
    % 初始化该 system 的存储结构(单车系统)
    ddl{n} = NaN(1, 1);
    T_est{n} = NaN(1, 1);
    pos_map{n} = NaN(1, 1);
    % ===== 定义 ddl 和 pos_map =====
    chain = pathInfo_agent_chain{n}{kn};
    posi = find(chain == agent_i, 1, 'first');  % 找到该 agent 在 chain 中的索引
    pos_map{n}(kn) = posi;
    pos_int = (posi+1)/2; %(posi+1)/2 第几个经过的int 1,2
    routeLabels(n) = pathInfo{n}.routeId(pos_int); %画图用
    % ===== extract 本地 execution time =======
    routeIdx = pathInfo{n}.routeId(pos_int); %当前系统的route idx 

    space_seq = IntSpaceDB{agent_i}.routeSpace{routeIdx};
    dur_seq = IntSpaceDB{agent_i}.routeDur{routeIdx};
    L = min(S, numel(dur_seq));
    Cmat(1:L, n) = dur_seq(1:L);
    MapMat(1:L, n) = space_seq(1:L);

    if posi == 1
       ddl{n}(kn) = alpha_tilde{n}(kn);
       arrival_ref(n) = alpha_tilde{n}(kn);
    else
       prev_road = chain(posi - 1);  % 前一个road
       prev_int  = chain(posi - 2);  % 前一个intersection
       x1 = x_prev{prev_road}{n}(kn) + Dt; x2 = x_prev{agent_i}{n}(kn); lambda = 0.5;
       ddl{n}(kn) = x1 + lambda*(x2-x1);
       % ddl{n}(kn) = (x_prev{prev_road}{n}(kn) + Dt + x_prev{agent_i}{n}(kn)) / 2; %non-first int 前面一定是road
       arrival_ref(n) = y_prev{prev_int}{n}(kn) + Dt; 
    end
    % 记录当前系统 n 的第一个参与任务的时间
    % d(n) = arrival_ref(n);
    d(n) = ddl{n}(kn);
    % ===== 直接给周期一个足够大的常数 =====
    T_est{n}(kn) = T_cst;
end    

%% initialization 
r = zeros(S,N); %remaining time at tw- 
ra_reset = zeros(S,N); % ra triggered by resetting rule 
o = zeros(1,N);
U_c = zeros(N,M); %To check contention 
U = zeros(N,M); %selector variable  
tw = 0;
gamma = cell(1, N);
alpha = cell(1, N);
x = cell(N,1); %记录位于tw,tw1之间的resource allocation
for n = 1:N
    x{n} = cell(1, 1);    
end
speed = cell(1, N);
%speed = zeros(N,max(NI)); %speed 
% for n = 1:N
%     alpha{n} = NaN(1, I(n));  % 所有任务初始化为 NaN
%     gamma{n} = NaN(1, I(n));  % 所有任务初始化为 NaN
% end
%-------------------------Nodes initialization-----------------------------
l = 1; %index of node
g = 0;
ni = zeros(1,N); %initialize all tasks index
f = 0;
NODES_cap = 2000;
NODES = cell(NODES_cap, 1);
NODES{1} = {l,d,r,o,tw,ni,0,U_c,U,g,gamma,f,speed,ra_reset,x,alpha,zeros(N,N),zeros(1,N)};
node_count = 1;
%initial node: l(index),ddl,remain,response,tw,ni,l(parentnode), V_c, V_past, g_cost,
%  {17} = priority_lock (1xM, 0=no priority established for that column)
OPEN = l;
LEAF = []; %record leaf node
c_node_index = l;

total_pruned = 0;
timeout_s = 60;
if isfield(const, 'timeout_int_s'), timeout_s = const.timeout_int_s; end
t_start = tic;

ctx = struct();
ctx.agent_i = agent_i;
ctx.M = M; ctx.S = S;
ctx.NI_agent = NI_agent;
ctx.entries = entries;
ctx.pos_map = pos_map;
ctx.T_est = T_est;
ctx.valid_systems = valid_systems;
ctx.Cmat = Cmat; 
ctx.MapMat = MapMat;
ctx.ddl = ddl;
ctx.arrival_ref = arrival_ref;
%% ---------------解包----------------
M = ctx.M;
NI_agent = ctx.NI_agent;
entries = ctx.entries;
pos_map      = ctx.pos_map;
T_est        = ctx.T_est;
valid_systems= ctx.valid_systems;
Cmat         = ctx.Cmat;
MapMat       = ctx.MapMat;   
ddl          = ctx.ddl;
arrival_ref  = ctx.arrival_ref;


% T_bound: worst-case sequential deadline for this intersection
total_C_local = sum(sum(Cmat(:, valid_systems)));
T_bound = max(arrival_ref(valid_systems)) + total_C_local;
fprintf('  [T_bound] Agent %d: %.2f\n', agent_i, T_bound);

% ── FUTURE: Parallel tree expansion ──────────────────────────────────────
% Current: expand one OPEN node per iteration (A*-style, pick min f-cost).
% Idea A (easy): parallelize LEAF solving in IN_Admm — each leaf's YALMIP
%   solve is independent. Change "for i=1:length(LEAF)" to parfor there.
%   Blocked by MATLAB nested-parfor limit: the 4 intersections already run
%   inside parfeval workers, so inner parfor degrades to serial.
% Idea B (batch BFS): each iteration expands ALL current OPEN nodes in
%   parallel (parfor), then merges child nodes into NODES. Requires
%   pre-allocating index ranges to avoid write conflicts.
% Idea C (flat parallelism): dissolve outer intersection-level parfeval;
%   instead build one flat task list of (intersection × leaf) pairs and
%   run a single parfor across all of them. Most efficient but largest
%   refactor.
% ─────────────────────────────────────────────────────────────────────────
iter_count = 0;
max_open = 0;
while any(ni <= NI_agent)
    iter_count = iter_count + 1;
    if length(OPEN) > max_open, max_open = length(OPEN); end
    if mod(iter_count, 200) == 0
        c_ni = NODES{c_node_index}{6};
        fprintf('  [Agent %d] iter=%d  OPEN=%d  LEAF=%d  nodes=%d  ni=%s  t=%.1fs\n', ...
            agent_i, iter_count, length(OPEN), length(LEAF), node_count, ...
            mat2str(c_ni(valid_systems)'), toc(t_start));
    end
    [NODES,OPEN,LEAF,np,node_count] = expand_array_IN(NODES,OPEN,c_node_index,LEAF,...
       ctx,const,node_count);
    total_pruned = total_pruned + np;

    %------------------PRUNE NODES (optional)-------------------
    if isfield(const,'use_pruning') && const.use_pruning && length(OPEN) > 1
        if isfield(const,'use_fast_prune') && const.use_fast_prune
            OPEN = prune_nodes_by_ni_fast(NODES, OPEN);
        else
            OPEN = prune_nodes_by_ni(NODES, OPEN);
        end
    end
    %--------------------T_bound pruning------------------------
    if isfield(const,'useTBound') && const.useTBound
        over = OPEN(arrayfun(@(idx) NODES{idx}{5} > T_bound, OPEN));
        if ~isempty(over)
            fprintf('  [T_bound] Agent %d: pruned %d nodes (tw > %.2f)\n', agent_i, numel(over), T_bound);
            OPEN = OPEN(~ismember(OPEN, over));
            if isempty(OPEN) && isempty(LEAF), LEAF = [LEAF, c_node_index]; end
        end
    end
    %------------------------------------------------------------
    if toc(t_start) > timeout_s
        fprintf('  [WeakRule] Agent %d: timeout %.1fs, forcing termination\n', agent_i, timeout_s);
        OPEN = [];
        if isempty(LEAF), LEAF = l; end
    end
    if ~isempty(OPEN)
        %find out the node with min f cost
        [~, minIndex] = f_min(NODES,OPEN);
        c_node_index = minIndex;
     else
        fprintf('  [Agent %d] nodes=%d  leaves=%d  max_OPEN=%d  pruned=%d  t=%.2fs\n', ...
            agent_i, node_count, length(LEAF), max_open, total_pruned, toc(t_start));
        NODES = NODES(1:node_count);
        [x, y,best_alpha,best_gamma,best_idx,NODES] ...
            = IN_Admm(NODES,LEAF,agent_i, entries,...
                x_prev{agent_i}, y_prev{agent_i}, xi_prev_bar, yi_prev_bar, ai_x, ai_y...
                , valid_systems, MapMat, Cmat,const);
        break;
        % %------ used to draw local schedules-------------
        % [min_f, minIndex] = f_min(NODES,LEAF); %compare costs of all leaves
        % c_node_index = minIndex; break;
     end

end

% ===== reconstruct chosen path from decision tree =====
resultNodes = [];
current_node_idx = best_idx;   % 被采用的 leaf

while current_node_idx > 0
    c_node = NODES{current_node_idx};
    resultNodes = [current_node_idx, resultNodes];
    current_node_idx = c_node{7};   % parent index
end

Path_min = resultNodes;
Optimal_g_cost = NODES{Path_min(end)}{10};

LocalTreeCache.NODES  = NODES;
LocalTreeCache.Path  = Path_min;
LocalTreeCache.best_idx = best_idx;
LocalTreeCache.cost  = Optimal_g_cost;

LocalTreeCache.Cmat  = Cmat;
LocalTreeCache.MapMat = MapMat;
LocalTreeCache.pos_map = pos_map;
LocalTreeCache.valid_systems = valid_systems;
LocalTreeCache.entries = entries;

% LocalTreeCache{agent_i}.NODES  = NODES;
% LocalTreeCache{agent_i}.Path  = Path_min;
% LocalTreeCache{agent_i}.best_idx = best_idx;
% LocalTreeCache{agent_i}.cost  = Optimal_g_cost;
% 
% LocalTreeCache{agent_i}.Cmat  = Cmat;
% LocalTreeCache{agent_i}.MapMat = MapMat;          % 如果 plot 里要用也存
% LocalTreeCache{agent_i}.pos_map = pos_map;        
% LocalTreeCache{agent_i}.valid_systems = valid_systems;  % 每个agent可能不同，别用全局
% LocalTreeCache{agent_i}.entries = entries;        % 可选：你画 route label/调试可能用

%% ---------------record optimal path--------------------------
% save('nodes.mat', 'NODES');
% 
% doPlot = true;
% if doPlot
%     plot_local_tree_schedule(NODES, Path_min, Cmat, agent_i, valid_systems, pathInfo, pos_map, ...
%         't1', 11.75, 'tf', 50, ...
%         'showReset', true, ...
%         'routeLabelMode', 'routeId');
% end
end
