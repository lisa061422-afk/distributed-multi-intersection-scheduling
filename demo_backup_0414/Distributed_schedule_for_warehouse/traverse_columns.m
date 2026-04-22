function [U_valid, n_pruned, cb_updates] = traverse_columns(U_c, priority_n, pair_lock, ra)
% Enumerate selector matrices only over contended columns.
%
% pair_lock : N×N matrix.  pair_lock(i,j) = winner (i or j) means i and j
%             have previously competed and winner won.  0 = never competed.
%             Weak rule: if one candidate beats ALL other contenders via
%             established pairwise locks AND is actively mid-task (ra>0),
%             that candidate auto-wins (no extra branch).
% ra        : S×N remaining-time matrix.
% n_pruned  : branches skipped by the weak rule.
% cb_updates: N×N matrix of forced pair_lock overwrites from cycle-break decisions.

    N = size(U_c, 1);
    if nargin < 2 || isempty(priority_n),  priority_n = 0;           end
    if nargin < 3 || isempty(pair_lock),   pair_lock  = zeros(N, N); end
    if nargin < 4 || isempty(ra),          ra         = [];           end

    n_pruned       = 0;
    cb_updates     = zeros(size(pair_lock));
    U_base         = zeros(size(U_c));
    contended_cols = [];

    for m = 1:size(U_c, 2)
        rows = find(U_c(:,m) > 0);

        if isempty(rows)
            continue

        elseif numel(rows) == 1
            U_base(rows, m) = U_c(rows, m);

        else
            % --- priority_n override ---
            if priority_n > 0 && U_c(priority_n, m) > 0
                U_base(priority_n, m) = U_c(priority_n, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                continue;
            end

            % --- weak rule: find a candidate that beats ALL others via
            %     established pairwise locks and is actively using m ---
            winner_lock = 0;
            if ~isempty(ra)
                for ci = 1:numel(rows)
                    candidate = rows(ci);
                    s_cand    = U_c(candidate, m);
                    if s_cand < 1 || s_cand > size(ra,1) || ra(s_cand, candidate) <= 1e-5
                        continue;
                    end
                    beats_all = true;
                    for oi = 1:numel(rows)
                        other = rows(oi);
                        if other == candidate, continue; end
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
                U_base(winner_lock, m) = U_c(winner_lock, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                % fprintf('[AUTO-WIN] space%d: N%d wins over N%s\n', m, winner_lock, mat2str(rows(rows~=winner_lock)'));
                continue;
            end

            % Cycle-break: if pair_lock among contestants forms a cycle,
            % break it deterministically.
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
                winner_cb   = min(candidates);
                U_base(winner_cb, m) = U_c(winner_cb, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                losers_cb = rows(rows ~= winner_cb);
                cb_updates(winner_cb, losers_cb) = winner_cb;
                cb_updates(losers_cb, winner_cb) = winner_cb;
                % fprintf('[CYCLE-BREAK] space%d: N%d wins over N%s\n', m, winner_cb, mat2str(losers_cb'));
                continue;
            end

            contended_cols(end+1) = m; %#ok<AGROW>
        end
    end

    if isempty(contended_cols)
        U_valid = {U_base};
        return;
    end

    U_valid = recurse_contended(U_c, U_base, contended_cols, 1, {});
end


function U_list = recurse_contended(U_c, U_temp, contended_cols, k, U_list)
    if k > numel(contended_cols)
        U_list{end+1} = U_temp;
        return;
    end
    m    = contended_cols(k);
    rows = find(U_c(:,m) > 0);
    for ii = 1:numel(rows)
        n      = rows(ii);
        U_next = U_temp;
        U_next(:, m) = 0;
        U_next(n, m) = U_c(n, m);
        U_list = recurse_contended(U_c, U_next, contended_cols, k+1, U_list);
    end
end

%--------------------------------------------------------------------------
function found = cycle_exists(pair_lock, rows)
    n   = numel(rows);
    in_deg = zeros(1, n);
    for i = 1:n
        for j = 1:n
            if i ~= j && pair_lock(rows(i), rows(j)) == rows(j)
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
            if pair_lock(rows(v), rows(u)) == rows(v)
                in_deg(u) = in_deg(u) - 1;
                if in_deg(u) == 0
                    queue(end+1) = u; %#ok<AGROW>
                end
            end
        end
    end
    found = (processed < n);
end
