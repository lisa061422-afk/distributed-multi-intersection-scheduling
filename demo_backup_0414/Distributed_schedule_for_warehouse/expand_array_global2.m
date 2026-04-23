function [NODES, OPEN, LEAF] = expand_array_global2(NODES, OPEN, c_node_index, LEAF, const)

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
        da(n)    = BIG_M;          % active task; travel countdown disabled until finish
        ra(:,n)  = 0;
        ra(1:length(task.C),n) = task.C(:);
        oa(n)    = 0;

        new_task_flag(n) = true;

        % record generation time
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
active_systems = find(any(ra > 1e-9, 1));

for n = active_systems
    s_idx = find(ra(:,n) > 1e-9, 1, 'first');
    if isempty(s_idx)
        continue;
    end
    task = getTaskProfile(n, ni2(n), const);
    m1 = task.Map(s_idx);
    U_c(n, m1) = s_idx;
end


%% ============================================================
% FCFS single branch with correct waiting/entry logic:
% 1) tasks already executing continue
% 2) released-but-not-started tasks remain waiting (not discarded)
% 3) a waiting task can start only if every currently executing task
%    in the same intersection has released all shared conflict spaces
% 4) among simultaneously admissible waiting tasks, use FCFS order:
%    first by generation time alpha, then by system index
%% ============================================================
U_temp = zeros(N, Mtot);
ra_temp = ra;

%% -------- 1) systems already executing must continue ----------
executing_now = [];

for n = 1:N
    if is_started_not_finished_local(n, ni2, ra, const)
        executing_now = [executing_now, n];
    end
end

for j = executing_now
    s_idx_j = find(ra(:,j) > 1e-9, 1, 'first');
    if isempty(s_idx_j)
        continue;
    end

    task_j = getTaskProfile(j, ni2(j), const);
    m_j = task_j.Map(s_idx_j);
    U_temp(j, m_j) = s_idx_j;
end

%% -------- 2) collect waiting candidates (released but not started) ----------
candidate_waiting = [];

for n = 1:N
    if ni2(n) <= 0
        continue;
    end

    if ~any(ra(:,n) > 1e-9)
        continue;
    end

    if is_started_not_finished_local(n, ni2, ra, const)
        continue;   % already executing, handled above
    end

    % this task is active but has not yet entered execution
    candidate_waiting = [candidate_waiting, n];
end

% sort FCFS:
% first by generation time of current task; tie-break by system index
if ~isempty(candidate_waiting)
    key = zeros(length(candidate_waiting), 2);
    for k = 1:length(candidate_waiting)
        n = candidate_waiting(k);
        if ni2(n) > 0 && iscell(alpha) && numel(alpha) >= n ...
                && numel(alpha{n}) >= ni2(n) && ~isnan(alpha{n}(ni2(n)))
            key(k,1) = alpha{n}(ni2(n));
        else
            key(k,1) = inf;
        end
        key(k,2) = n;
    end
    [~, order] = sortrows(key, [1 2]);
    candidate_waiting = candidate_waiting(order);
end

%% -------- 3) admit waiting tasks only when shared spaces released ----------
accepted_waiting = [];

for ii = 1:numel(candidate_waiting)
    n = candidate_waiting(ii);

    s_idx_n = find(ra(:,n) > 1e-9, 1, 'first');
    if isempty(s_idx_n)
        continue;
    end

    task_n = getTaskProfile(n, ni2(n), const);
    map_n  = task_n.Map(:)'; %哪个resource of 1-20
    m1_n   = task_n.Map(s_idx_n);

    can_start = true;

    % check against currently executing systems:
    % n can start only when every such system has released all
    % spaces shared with n
    for j = executing_now
        if j == n
            continue;
        end

        if ~has_released_shared_spaces_local(j, n, ni2, ra, const)
            can_start = false;
            break;
        end
    end

    if ~can_start
        continue;
    end

    % check against other waiting tasks already accepted at this tw
    % if they share any space in same intersection, later one must wait
    for kk = 1:length(accepted_waiting)
        j = accepted_waiting(kk);

        task_j = getTaskProfile(j, ni2(j), const);
        map_j  = task_j.Map(:)';

        int_j = ceil(map_j(1) / const.spacePerInt);
        int_n = ceil(map_n(1) / const.spacePerInt);

        if int_j ~= int_n
            continue;
        end

        map_j_valid = map_j(map_j > 0);
        map_n_valid = map_n(map_n > 0);
        shared_valid = intersect(map_j_valid, map_n_valid);

        if ~isempty(shared_valid)
            can_start = false;
            break;
        end
    end

    if ~can_start
        continue;
    end

    % first space cannot already be occupied this instant
    if any(U_temp(:,m1_n) > 1e-9)
        continue;
    end
    U_temp(n, m1_n) = s_idx_n;
    accepted_waiting = [accepted_waiting, n];
end

%% ---------------- advance to next significant moment ----------------
[d2, r2, o2, tw1] = NextSigM_global(tw, da, ra_temp, oa, U_temp, const);

% safeguard: no progress
if tw1 <= tw + 1e-12
    warning('No time progress at node %d: tw1 == tw == %.6f. Mark as leaf.', c_node_index, tw);
    LEAF = [LEAF, l];
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

end

%% ========================================================================
function flag = is_started_not_finished_local(n, ni_vec, r_mat, const)
% True if current task of system n has already started, but not finished.

flag = false;

if ni_vec(n) <= 0
    return;
end

if ~any(r_mat(:,n) > 1e-9)
    return;
end

task = getTaskProfile(n, ni_vec(n), const);
Csum = sum(task.C);
rsum = sum(r_mat(:,n));

% If remaining work is strictly between full C and zero,
% then execution has started and not finished yet.
if rsum > 1e-9 && rsum < Csum - 1e-9
    flag = true;
end

end

%% ========================================================================
function released = has_released_shared_spaces_local(j, n, ni_vec, r_mat, const)
% For executing system j and candidate system n:
% released = true only if j has already left all spaces shared with n.

released = true;

if ni_vec(j) <= 0 || ni_vec(n) <= 0
    return;
end

task_j = getTaskProfile(j, ni_vec(j), const);
task_n = getTaskProfile(n, ni_vec(n), const);

map_j = task_j.Map(:)';
map_n = task_n.Map(:)';

% different intersection -> irrelevant
int_j = ceil(map_j(1) / const.spacePerInt);
int_n = ceil(map_n(1) / const.spacePerInt);
if int_j ~= int_n
    return;
end

map_j_valid = map_j(map_j > 0);
map_n_valid = map_n(map_n > 0);
shared_spaces = intersect(map_j_valid, map_n_valid);
%shared_spaces = intersect(map_j, map_n);

% no shared space -> no blocking
if isempty(shared_spaces)
    return;
end

% identify j's current remaining path from its current subtask onward
s_idx_j = find(r_mat(:,j) > 1e-9, 1, 'first');
if isempty(s_idx_j)
    return;
end

remaining_spaces_j = map_j(s_idx_j:end);
remaining_spaces_j = remaining_spaces_j(remaining_spaces_j > 0);

shared_spaces = shared_spaces(shared_spaces > 0);

% if any shared space is still in j's remaining path,
% then j has NOT released all shared spaces yet
if ~isempty(intersect(shared_spaces, remaining_spaces_j))
    released = false;
end

end