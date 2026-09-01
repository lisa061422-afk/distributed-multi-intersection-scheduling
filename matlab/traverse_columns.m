function [U_valid, n_pruned] = traverse_columns(U_c, priority_n, priority_lock, ra)
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
    if nargin < 3 || isempty(priority_lock), priority_lock = zeros(1,size(U_c,2)); end
    if nargin < 4 || isempty(ra),            ra            = [];                   end

    n_pruned       = 0;
    U_base         = zeros(size(U_c));
    contended_cols = [];

    for m = 1:size(U_c, 2)
        rows = find(U_c(:,m) > 0);

        if isempty(rows)
            continue;

        elseif numel(rows) == 1
            U_base(rows, m) = U_c(rows, m);

        else
            % --- existing priority_n override ---
            if priority_n > 0 && U_c(priority_n, m) > 0
                U_base(priority_n, m) = U_c(priority_n, m);
                n_pruned = n_pruned + (numel(rows) - 1);
                continue;
            end

            % --- weak rule: locked vehicle actively using m wins automatically ---
            locked_n = priority_lock(m);
            if locked_n > 0 && any(rows == locked_n) && ~isempty(ra)
                s_locked = U_c(locked_n, m);      % subtask index stored in U_c
                if s_locked >= 1 && s_locked <= size(ra,1) && ra(s_locked, locked_n) > 1e-5
                    U_base(locked_n, m) = U_c(locked_n, m);
                    n_pruned = n_pruned + (numel(rows) - 1);
                    continue;
                end
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
