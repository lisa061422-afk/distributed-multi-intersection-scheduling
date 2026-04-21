function [V_valid, n_pruned, cb_updates] = traverse_columns(V_c, priority_n, pair_lock, ra)
% Enumerate selector matrices only over contended columns.
%
% pair_lock : N×N matrix.  pair_lock(i,j) = winner (i or j) means i and j
%             have previously competed and winner won.  0 = never competed.
%             Weak rule: if one candidate beats ALL other contenders via
%             established pairwise locks AND is actively mid-task (ra>0),
%             that candidate auto-wins (no extra branch).
% ra        : S×N remaining-time matrix.
% n_pruned  : branches skipped by the weak rule.

    N = size(V_c, 1);
    if nargin < 2 || isempty(priority_n),  priority_n = 0;           end
    if nargin < 3 || isempty(pair_lock),   pair_lock  = zeros(N, N); end
    if nargin < 4 || isempty(ra),          ra         = [];           end

    n_pruned       = 0;
    cb_updates     = zeros(size(pair_lock));  % forced overwrites from cycle-break decisions
    V_base         = zeros(size(V_c));
    contended_cols = [];

    for m = 1:size(V_c, 2)
        rows = find(V_c(:,m) > 0);

        if isempty(rows)
            continue;

        elseif numel(rows) == 1
            V_base(rows, m) = V_c(rows, m);

        else
            % --- priority_n override ---
            if priority_n > 0 && V_c(priority_n, m) > 0
                V_base(priority_n, m) = V_c(priority_n, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                continue;
            end

            % --- weak rule: find a candidate that beats ALL others via
            %     established pairwise locks and is actively using m ---
            winner_lock = 0;
            if ~isempty(ra)
                for ci = 1:numel(rows)
                    candidate = rows(ci);
                    s_cand    = V_c(candidate, m);
                    if s_cand < 1 || s_cand > size(ra,1) || ra(s_cand, candidate) <= 1e-5
                        continue;  % not actively mid-task on m
                    end
                    beats_all = true;
                    for oi = 1:numel(rows)
                        other = rows(oi);
                        if other == candidate, continue; end
                        % pair_lock(candidate,other) must equal candidate
                        % (established and candidate won); 0 = never competed
                        if pair_lock(candidate, other) ~= candidate
                            beats_all = false;
                            break;
                        end
                    end
                    if beats_all
                        winner_lock = candidate;
                        break;
                    end
                end
            end

            if winner_lock > 0
                V_base(winner_lock, m) = V_c(winner_lock, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                fprintf('[AUTO-WIN] space%d: N%d wins over N%s\n', m, winner_lock, mat2str(rows(rows~=winner_lock)'));
                continue;
            end

            % Cycle-break: if pair_lock among contestants forms a cycle,
            % no topological winner exists — break it deterministically.
            if cycle_exists(pair_lock, rows)
                win_count = zeros(1, numel(rows));
                for ci = 1:numel(rows)
                    for oi = 1:numel(rows)
                        if ci ~= oi && pair_lock(rows(ci), rows(oi)) == rows(ci)
                            win_count(ci) = win_count(ci) + 1;
                        end
                    end
                end
                [max_w, ~] = max(win_count);
                candidates  = rows(win_count == max_w);
                winner_cb   = min(candidates);  % lowest index breaks tie
                V_base(winner_cb, m) = V_c(winner_cb, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                % Force-overwrite pair_lock for all cycle members so cycle can't recur
                losers_cb = rows(rows ~= winner_cb);
                cb_updates(winner_cb, losers_cb) = winner_cb;
                cb_updates(losers_cb, winner_cb) = winner_cb;
                fprintf('[CYCLE-BREAK] space%d: N%d wins over N%s\n', m, winner_cb, mat2str(losers_cb'));
                continue;
            end

            contended_cols(end+1) = m; %#ok<AGROW>
        end
    end

    if isempty(contended_cols)
        V_valid = {V_base};
        return;
    end

    V_valid = recurse_contended(V_c, V_base, contended_cols, 1, {});
end


function V_list = recurse_contended(V_c, V_temp, contended_cols, k, V_list)
% Recursively enumerate all winner assignments for contended spaces.
% V_c           : original contention matrix (read-only)
% V_temp        : assignment built so far (inherited from parent call)
% contended_cols: spaces that still need a winner assigned
% k             : index into contended_cols — current recursion depth
% V_list        : accumulator of completed assignment matrices

    % base case: all contended spaces assigned → save this assignment
    if k > numel(contended_cols)
        V_list{end+1} = V_temp;
        return;
    end

    m    = contended_cols(k);      % current contended space
    rows = find(V_c(:,m) > 0);    % systems competing for space m

    for ii = 1:numel(rows)
        n      = rows(ii);         % try system n as winner of space m
        V_next = V_temp;
        V_next(:, m) = 0;          % clear any previous assignment for space m
        V_next(n, m) = V_c(n, m); % assign space m to system n (with sub-task index)
        V_list = recurse_contended(V_c, V_next, contended_cols, k+1, V_list);
    end
end

%--------------------------------------------------------------------------
function found = cycle_exists(pair_lock, rows)
% Returns true if the pair_lock dominance graph on 'rows' contains a cycle.
% Uses Kahn's algorithm (topological sort via in-degree).
    n   = numel(rows);
    in_deg = zeros(1, n);
    for i = 1:n
        for j = 1:n
            if i ~= j && pair_lock(rows(i), rows(j)) == rows(j)
                % rows(j) dominates rows(i): edge j→i, so in_deg(i)++
                in_deg(i) = in_deg(i) + 1;
            end
        end
    end
    queue = find(in_deg == 0);
    processed = 0;
    while ~isempty(queue)
        v = queue(1);  queue = queue(2:end);
        processed = processed + 1;
        for u = 1:n
            if pair_lock(rows(v), rows(u)) == rows(v)  % v dominates u
                in_deg(u) = in_deg(u) - 1;
                if in_deg(u) == 0
                    queue(end+1) = u; %#ok<AGROW>
                end
            end
        end
    end
    found = (processed < n);
end
