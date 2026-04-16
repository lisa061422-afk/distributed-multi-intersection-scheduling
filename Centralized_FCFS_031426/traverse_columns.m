function U_valid = traverse_columns(U_c)
% Enumerate selector matrices only over contended columns.
% Non-contended columns are fixed directly.

    [N, M] = size(U_c); %#ok<ASGLU>

    U_base = zeros(size(U_c));
    contended_cols = [];

    for m = 1:M
        rows = find(U_c(:,m) > 0);

        if isempty(rows)
            continue;
        elseif numel(rows) == 1
            U_base(rows, m) = U_c(rows, m);
        else
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

    m = contended_cols(k);
    rows = find(U_c(:,m) > 0);

    for ii = 1:numel(rows)
        n = rows(ii);

        U_next = U_temp;
        U_next(:,m) = 0;
        U_next(n,m) = U_c(n,m);

        U_list = recurse_contended(U_c, U_next, contended_cols, k+1, U_list);
    end
end