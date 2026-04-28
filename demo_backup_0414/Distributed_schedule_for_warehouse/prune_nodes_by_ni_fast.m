function OPEN = prune_nodes_by_ni_fast(NODES, OPEN)
% Vectorized version of prune_nodes_by_ni: same semantics, no containers.Map,
% no mat2str.  Uses `unique(M, 'rows')` over an [|OPEN|, N] matrix of ni vectors.
%
% Eligibility (matches original): only nodes whose `r` is fully zero (i.e. the
% current sub-task is finished) are eligible for ni-based merging.  Ineligible
% nodes pass through unchanged.

    n = numel(OPEN);
    if n == 0, return; end

    % Bulk read fields (cell-of-cells access is unavoidable; loop is O(n))
    nN = numel(NODES{OPEN(1)}{6});
    eligible = false(1, n);
    f_vals   = zeros(1, n);
    ni_rows  = zeros(n, nN);
    for i = 1:n
        idx = OPEN(i);
        if all(NODES{idx}{3}(:) <= 1e-5)
            eligible(i)  = true;
            f_vals(i)    = NODES{idx}{12};
            ni_rows(i,:) = NODES{idx}{6};
        end
    end

    elig_idx = find(eligible);
    if numel(elig_idx) <= 1, return; end

    [~, ~, gid] = unique(ni_rows(elig_idx, :), 'rows');
    n_groups = max(gid);
    keep = true(1, numel(elig_idx));
    for g = 1:n_groups
        members = find(gid == g);
        if numel(members) > 1
            [~, mi] = min(f_vals(elig_idx(members)));
            keep(members) = false;
            keep(members(mi)) = true;
        end
    end

    % Build the survivor mask over the original OPEN, then return in the same
    % sorted-unique order setdiff() would produce, so the downstream f_min /
    % expansion order is identical to the original prune_nodes_by_ni.
    keep_mask = true(1, n);
    keep_mask(elig_idx(~keep)) = false;
    OPEN = unique(OPEN(keep_mask));   % unique is sorted-ascending → matches setdiff
end
