function [NODES, OPEN, LEAF] = expand_array_global(NODES, OPEN, c_node_index, LEAF, const)

N     = const.N;
NI    = const.NI;
Smax  = const.Smax;
Mtot  = const.Mtot;
BIG_M = const.BIG_M;

c_node = NODES{c_node_index};

l      = c_node{1};
d      = c_node{2};
r      = c_node{3};
o      = c_node{4};
tw     = c_node{5};
ni     = c_node{6};
g      = c_node{10};
gamma  = c_node{11};
speed  = c_node{13};
x      = c_node{15};
alpha  = c_node{16};

current_node_index = l;
OPEN(OPEN == current_node_index) = [];

%% ---------------- initialize updated states at tw ----------------
da  = inf(1,N);
ra  = zeros(Smax,N);
oa  = zeros(1,N);
ni2 = ni;
U_c = zeros(N,Mtot);

% mark which tasks are newly released exactly at this tw
new_task_flag = false(1,N);

%% ---------------- from tw- to tw: release new tasks if d<=0 ----------------
for n = 1:N

    % all tasks finished
    if ni(n) >= NI(n) && all(r(:,n) <= 1e-9)
        ni2(n)   = NI(n);
        da(n)    = Inf;
        ra(:,n)  = 0;
        oa(n)    = 0;
        continue;
    end

    % a new task is released at tw
    if d(n) <= 1e-9 && ni(n) < NI(n)
        nextTask = ni(n) + 1;
        task = getTaskProfile(n, nextTask, const);

        ni2(n)   = nextTask;
        da(n)    = BIG_M;          % task is active now
        ra(:,n)  = 0;
        ra(1:length(task.C),n) = task.C(:);
        oa(n)    = 0;

        new_task_flag(n) = true;

        % record generation time if alpha node exists
        if iscell(alpha) && numel(alpha) >= n
            if isempty(alpha{n})
                alpha{n} = NaN(1, NI(n));
            end
            if numel(alpha{n}) < NI(n)
                alpha{n}(NI(n)) = NaN;
            end
            if isnan(alpha{n}(nextTask))
                alpha{n}(nextTask) = tw;
            end
        end

    else
        % keep current state
        ni2(n)   = ni(n);
        da(n)    = d(n);
        ra(:,n)  = r(:,n);
        oa(n)    = o(n);
    end
end

%% ---------------- terminate if all done ----------------
all_done = true;
for n = 1:N
    if ~(ni2(n) >= NI(n) && all(ra(:,n) <= 1e-9))
        all_done = false;
        break;
    end
end

if all_done
    LEAF = [LEAF, l];
    return;
end

%% ---------------- build raw request matrix U_c ----------------
active_systems = find(any(ra > 1e-5, 1));

for n = active_systems
    s_idx = find(ra(:,n) > 1e-5, 1, 'first');
    if isempty(s_idx)
        continue;
    end

    task = getTaskProfile(n, ni2(n), const);
    m1 = task.Map(s_idx);
    U_c(n, m1) = s_idx;
end

%% ---------------- debug print ----------------
disp('---------------- expand_array_global (FCFS single-branch) ----------------');
disp(['current node = ', num2str(c_node_index)]);
disp('tw = '); disp(tw);
disp('ni  = '); disp(ni);
disp('ni2 = '); disp(ni2);
disp('da  = '); disp(da);
disp('ra  = '); disp(ra);
disp('r(tw-) = '); disp(r);
disp('oa  = '); disp(oa);
disp('new_task_flag = '); disp(new_task_flag);
disp('U_c = '); disp(U_c);

%% =======================================================
%  FCFS single branch:
%  1) old tasks in r(tw-) continue first
%  2) new tasks at current tw can start only if not blocked
% =======================================================
U_temp = zeros(N, Mtot);
ra_temp = ra;

% old active tasks at tw- : they are already executing, so continue
% old_active = find(any(r > 1e-5, 1));
% old active tasks at tw- : task has started but not finished
old_active = [];

for n = 1:N
    if ~any(r(:,n) > 1e-5)
        continue;
    end

    task_old = getTaskProfile(n, ni(n), const);

    r_sum = sum(r(:,n));
    C_sum = sum(task_old.C);

    if r_sum > 1e-5 && r_sum < C_sum - 1e-5
        old_active = [old_active, n];
    end
end

for n = old_active
    s_idx = find(ra(:,n) > 1e-5, 1, 'first');
    if isempty(s_idx)
        continue;
    end

    task = getTaskProfile(n, ni2(n), const);
    m1 = task.Map(s_idx);
    U_temp(n, m1) = s_idx;
end

% new tasks released exactly at current tw
new_tasks = find(new_task_flag);

% FCFS among simultaneous new tasks:
% since they are released at the same tw, use system index order
accepted_new = [];

for ii = 1:length(new_tasks)
    n = new_tasks(ii);

    % safety
    if ~any(ra(:,n) > 1e-5)
        continue;
    end

    s_idx = find(ra(:,n) > 1e-5, 1, 'first');
    task_n = getTaskProfile(n, ni2(n), const);
    map_n = task_n.Map(:)';
    m1_n = task_n.Map(s_idx);

    can_start = true;

    % ---------------------------------------------------
    % check against old tasks already executing at tw-
    % use old r to identify where they are
    % ---------------------------------------------------
    for j = old_active
        if ~any(r(:,j) > 1e-5)
            continue;
        end

        task_j = getTaskProfile(j, ni(j), const);
        map_j = task_j.Map(:)';

        % same intersection only
        if ceil(map_j(1)/const.spacePerInt) ~= ceil(map_n(1)/const.spacePerInt)
            continue;
        end

        s_idx_j = find(r(:,j) > 1e-5, 1, 'first');
        map_j_rem = map_j(s_idx_j:end);

        % if old task remaining path overlaps new task path, new task cannot start
        if ~isempty(intersect(map_j_rem, map_n))
            can_start = false;
            break;
        end
    end

    if ~can_start
        continue;
    end

    % ---------------------------------------------------
    % check against new tasks already accepted at this tw
    % ---------------------------------------------------
    for kk = 1:length(accepted_new)
        j = accepted_new(kk);

        task_j = getTaskProfile(j, ni2(j), const);
        map_j = task_j.Map(:)';

        % same intersection only
        if ceil(map_j(1)/const.spacePerInt) ~= ceil(map_n(1)/const.spacePerInt)
            continue;
        end

        % if two simultaneous new tasks overlap, only keep the earlier one
        if ~isempty(intersect(map_j, map_n))
            can_start = false;
            break;
        end
    end

    if ~can_start
        continue;
    end

    % ---------------------------------------------------
    % also respect current first-space occupancy
    % ---------------------------------------------------
    if any(U_temp(:, m1_n) > 0)
        continue;
    end

    U_temp(n, m1_n) = s_idx;
    accepted_new = [accepted_new, n];
end

disp('U_temp (FCFS) = ');
disp(U_temp);

% if nnz(U_temp) == 0
%     fprintf('No executable FCFS action at node %d. Mark as leaf.\n', c_node_index);
%     LEAF = [LEAF, current_node_index];
%     return;
% end
%% ---------------- advance to next significant moment ----------------
[d2, r2, o2, tw1] = NextSigM_global(tw, da, ra_temp, oa, U_temp, const);
if abs(tw1 - tw) < 1e-9
    warning('No time progress at node %d: tw1 == tw == %.6f. Mark as leaf.', ...
        c_node_index, tw);
    LEAF = [LEAF, current_node_index];
    return;
end
ra_reset = -1 * ones(Smax, N);

num_nodes = size(NODES, 1);
NODES_new = NewNode_global(num_nodes, d2, r2, o2, tw1, ni2, ...
    current_node_index, U_c, U_temp, g, gamma, speed, ra_temp, ra_reset, x, alpha, const);

OPEN  = [OPEN, NODES_new{1}];
NODES = [NODES; {NODES_new}];

fprintf('Generated one FCFS child node.\n');
fprintf('new tw = %.6f\n', tw1);

return;