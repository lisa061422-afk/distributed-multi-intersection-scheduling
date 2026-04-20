function [V_valid, n_pruned] = traverse_columns(V_c, priority_n, pair_lock, ra)
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
    if k > numel(contended_cols)
        V_list{end+1} = V_temp;
        return;
    end
    m    = contended_cols(k);
    rows = find(V_c(:,m) > 0);
    for ii = 1:numel(rows)
        n      = rows(ii);
        V_next = V_temp;
        V_next(:, m) = 0;
        V_next(n, m) = V_c(n, m);
        V_list = recurse_contended(V_c, V_next, contended_cols, k+1, V_list);
    end
end
