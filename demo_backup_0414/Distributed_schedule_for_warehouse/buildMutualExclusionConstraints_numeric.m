function [Aineq, bineq] = buildMutualExclusionConstraints_numeric(...
    MapMat, Cmat, valid_systems, gamma, idx_map, C_vec, eps_gap)
% Numeric equivalent of addMutualExclusion_SpaceLevel_Nonpreemptive.
% Returns [Aineq, bineq] for quadprog: Aineq * x_vec <= bineq
% where x_vec is indexed over valid_systems via idx_map.
%
% Constraint for adjacent pair (k1 before k2) in each contended space:
%   e(k1) + eps <= s(k2)
%   => x(n1) - x(n2) <= (C_n2 - tail2 - C(s2,n2)) - (C_n1 - tail1) - eps

nv  = numel(valid_systems);
Ntot = size(MapMat, 2);

ghat = zeros(Ntot, 1);
if iscell(gamma)
    for n = valid_systems
        if ~isempty(gamma{n}), ghat(n) = gamma{n}(1); end
    end
else
    for n = valid_systems, ghat(n) = gamma(n); end
end

all_spaces = [];
for n = valid_systems
    seq = MapMat(:,n);
    all_spaces = [all_spaces; seq(seq > 0)];
end
uniq_spaces = unique(all_spaces);
contended   = uniq_spaces(arrayfun(@(m) sum(all_spaces == m), uniq_spaces) >= 2);

Aineq = zeros(0, nv);
bineq = zeros(0, 1);

for mm = contended(:)'
    occ = [];
    for n = valid_systems
        s_list = find(MapMat(:, n) == mm);
        for t = 1:numel(s_list)
            occ = [occ; n, s_list(t)];
        end
    end
    K = size(occ, 1);
    if K <= 1, continue; end

    s_hat    = zeros(K, 1);
    tail_val = zeros(K, 1);
    for k = 1:K
        n = occ(k,1); s = occ(k,2);
        Sn       = find(MapMat(:,n) > 0, 1, 'last');
        tail_sum = sum(Cmat(s+1:Sn, n));
        tail_val(k) = tail_sum;
        e_hat    = ghat(n) - tail_sum;
        s_hat(k) = e_hat - Cmat(s, n);
    end

    [~, order] = sort(s_hat, 'ascend');

    for r = 1:K-1
        k1 = order(r);   n1 = occ(k1,1); s1 = occ(k1,2); %#ok<NASGU>
        k2 = order(r+1); n2 = occ(k2,1); s2 = occ(k2,2);

        % x(n1) - x(n2) <= rhs
        rhs = (C_vec(n2) - tail_val(k2) - Cmat(s2,n2)) ...
            - (C_vec(n1) - tail_val(k1)) - eps_gap;

        row = zeros(1, nv);
        i1 = idx_map(n1); i2 = idx_map(n2);
        if i1 > 0, row(i1) =  1; end
        if i2 > 0, row(i2) = -1; end

        Aineq(end+1, :) = row;   %#ok<AGROW>
        bineq(end+1)    = rhs;   %#ok<AGROW>
    end
end
end
