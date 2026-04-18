function [V_valid, n_pruned] = traverse_columns(V_c, priority_n, priority_lock, ra)
% Enumerate selector matrices only over contended columns.
% Non-contended columns are fixed directly.
%
% priority_n     : if > 0, that robot auto-wins any contended column.
% priority_lock  : 1xM vector; priority_lock(m) = vehicle n with priority
%                  for column m on this path (0 = none).
%                  Weak rule: if n is still actively using m (ra(s,n)>0),
%                  n wins automatically — no extra branch is created.
% ra             : SxN remaining-time matrix (needed for weak rule check).
% n_pruned       : number of branches skipped by the weak rule.

    if nargin < 2 || isempty(priority_n),    priority_n    = 0;                    end
    if nargin < 3 || isempty(priority_lock), priority_lock = zeros(1,size(V_c,2)); end
    if nargin < 4 || isempty(ra),            ra            = [];                   end

    n_pruned      = 0;
    V_base        = zeros(size(V_c));
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

            % --- weak rule: locked vehicle actively using m wins automatically ---
            locked_n = priority_lock(m);
            if locked_n > 0 && any(rows == locked_n) && ~isempty(ra)
                s_locked = V_c(locked_n, m);
                if s_locked >= 1 && s_locked <= size(ra,1) && ra(s_locked, locked_n) > 1e-5
                    V_base(locked_n, m) = V_c(locked_n, m);
                    n_pruned = n_pruned + (numel(rows) - 1);
                    continue;
                end
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
