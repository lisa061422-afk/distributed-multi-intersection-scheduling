function branches = resetting_rule(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,ctx,const,pair_lock,reset_since)
% Returns branches: cell array of {ra_temp, V_temp, x, pair_lock, reset_since, tw1}.
% Each interrupted system may generate multiple variants depending on whether
% it can displace V_temp occupants from the spaces it needs to re-enter.

N = const.N;  MapMat = ctx.MapMat;  Cmat = ctx.Cmat;

if nargin < 13 || isempty(pair_lock),   pair_lock   = zeros(N, N); end
if nargin < 14 || isempty(reset_since), reset_since = zeros(1, N); end

tw1 = tw1_basedon_U_temp(V_temp,ra,da,tw,const);
interrupted = find(any(ra > 0.00001, 1) & all(V_temp == 0, 2)');

% Start with one base branch
branches = {{ra_temp, V_temp, x, pair_lock, reset_since, tw1}};

for n = interrupted
    next_branches = {};
    for bi = 1:numel(branches)
        b      = branches{bi};
        ra_b   = b{1};  V_b  = b{2};  x_b  = b{3};
        pl_b   = b{4};  rs_b = b{5};

        x_b{n}{ni2(n)} = {};   % discard stale reservations
        rs_b(n)        = tw;   % mark ancestor V records stale from tw

        s_int = V_c(n, V_c(n,:) > 0);

        %------------------sub 1 interrupted-------------------------------
        if s_int == 1
            ra_b(:,n) = Cmat(:,n);
            pl_b      = relock_Vtemp(pl_b, V_b, n, MapMat(1,n));
            next_branches{end+1} = {ra_b, V_b, x_b, pl_b, rs_b, tw1}; %#ok<AGROW>

        %------------------sub 2 interrupted-------------------------------
        elseif s_int == 2
            a1       = MapMat(1,n);
            t1_start = tw1 - Cmat(1,n);
            valid_a1 = trace_valid_nodes(l, tw, t1_start, NODES);

            vars = space_variants(V_b, a1, n, pl_b, rs_b);
            for vi = 1:numel(vars)
                V_vi = vars{vi}{1};  displaced = vars{vi}{2};
                ra_vi = ra_b;  x_vi = x_b;  pl_vi = pl_b;  rs_vi = rs_b;

                if ~isempty(displaced)
                    np = displaced;
                    ra_vi(:,np)        = Cmat(:,np);
                    x_vi{np}{ni2(np)}  = {};
                    rs_vi(np)          = tw;
                    if pl_vi(n,np)==0 || pl_vi(n,np)==n
                        pl_vi(n,np) = n;  pl_vi(np,n) = n;
                    end
                    x_vi{n}{ni2(n)} = [x_vi{n}{ni2(n)}; {tw1-Cmat(1,n), tw1, 1, a1}];
                    ra_vi(1,n)      = 0;
                    ra_vi(2:end,n)  = Cmat(2:end,n);
                else
                    [~,tb1] = check_resc_occupation(NODES,valid_a1,a1,l,V_vi,n,tw1,tw,rs_vi);
                    [tb1, blkr] = check_x(tb1, 1, a1, x_vi, n, Cmat, N);
                    if blkr > 0 && blkr ~= n
                        if pl_vi(n,blkr)==0||pl_vi(n,blkr)==blkr
                            pl_vi(n,blkr)=blkr;  pl_vi(blkr,n)=blkr;
                        end
                    end
                    pl_vi = relock_Vtemp(pl_vi, V_vi, n, a1);
                    if tb1 <= t1_start + 1e-5
                        x_vi{n}{ni2(n)} = [x_vi{n}{ni2(n)}; {tw1-Cmat(1,n), tw1, 1, a1}];
                        ra_vi(1,n) = 0;
                    else
                        x_vi{n}{ni2(n)} = [x_vi{n}{ni2(n)}; {tb1, tb1+Cmat(1,n), 1, a1}];
                        ra_vi(1,n) = tb1 + Cmat(1,n) - tw1;
                    end
                    ra_vi(2:end,n) = Cmat(2:end,n);
                end
                % fprintf('[RR sub2] N%d vi=%d: ra1=%.3f ra2=%.3f\n', n, vi, ra_vi(1,n), ra_vi(2,n));
                next_branches{end+1} = {ra_vi, V_vi, x_vi, pl_vi, rs_vi, tw1}; %#ok<AGROW>
            end

        %------------------sub 3 interrupted-------------------------------
        elseif s_int == 3
            a2 = MapMat(2,n);  b1 = MapMat(1,n);
            t22      = tw1 - Cmat(2,n);
            valid_a2 = trace_valid_nodes(l, tw, t22, NODES);

            vars_a2 = space_variants(V_b, a2, n, pl_b, rs_b);
            for vi2 = 1:numel(vars_a2)
                V_v2 = vars_a2{vi2}{1};  disp2 = vars_a2{vi2}{2};
                ra_v2 = ra_b;  x_v2 = x_b;  pl_v2 = pl_b;  rs_v2 = rs_b;

                if ~isempty(disp2)
                    np2 = disp2;
                    ra_v2(:,np2)        = Cmat(:,np2);
                    x_v2{np2}{ni2(np2)} = {};
                    rs_v2(np2)          = tw;
                    if pl_v2(n,np2)==0||pl_v2(n,np2)==n
                        pl_v2(n,np2)=n;  pl_v2(np2,n)=n;
                    end
                    ta2_avail = tw1;
                else
                    [~,ta2] = check_resc_occupation(NODES,valid_a2,a2,l,V_v2,n,tw1,tw,rs_v2);
                    ta2_avail = max(ta2, t22);
                    [ta2_avail, blkr2] = check_x(ta2_avail, 2, a2, x_v2, n, Cmat, N);
                    if blkr2>0 && blkr2~=n
                        if pl_v2(n,blkr2)==0||pl_v2(n,blkr2)==blkr2
                            pl_v2(n,blkr2)=blkr2;  pl_v2(blkr2,n)=blkr2;
                        end
                    end
                    pl_v2 = relock_Vtemp(pl_v2, V_v2, n, a2);
                end

                t21          = ta2_avail - Cmat(1,n);
                [ta_bar_node,ta_bar] = find_node_ta_bar(l, tw, ta2_avail, NODES);
                valid_b1     = trace_valid_nodes(ta_bar_node, ta_bar, t21, NODES);

                vars_b1 = space_variants(V_v2, b1, n, pl_v2, rs_v2);
                for vi1 = 1:numel(vars_b1)
                    V_v1 = vars_b1{vi1}{1};  disp1 = vars_b1{vi1}{2};
                    ra_v1 = ra_v2;  x_v1 = x_v2;  pl_v1 = pl_v2;  rs_v1 = rs_v2;

                    if ~isempty(disp1)
                        np1 = disp1;
                        ra_v1(:,np1)        = Cmat(:,np1);
                        x_v1{np1}{ni2(np1)} = {};
                        rs_v1(np1)          = tw;
                        if pl_v1(n,np1)==0||pl_v1(n,np1)==n
                            pl_v1(n,np1)=n;  pl_v1(np1,n)=n;
                        end
                        x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {t21, ta2_avail, 1, b1}];
                        x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {ta2_avail, tw1, 2, a2}];
                        ra_v1(2,n) = ta2_avail + Cmat(2,n) - tw1;
                        ra_v1(3,n) = Cmat(3,n);
                    else
                        [~,tb1] = check_resc_occupation(NODES,valid_b1,b1,ta_bar_node,V_v1,n,tw1,tw,rs_v1);
                        [tb1, blkrb] = check_x(tb1, 1, b1, x_v1, n, Cmat, N);
                        if blkrb>0 && blkrb~=n
                            if pl_v1(n,blkrb)==0||pl_v1(n,blkrb)==blkrb
                                pl_v1(n,blkrb)=blkrb;  pl_v1(blkrb,n)=blkrb;
                            end
                        end
                        pl_v1 = relock_Vtemp(pl_v1, V_v1, n, b1);

                        if tb1 <= t21 + 1e-5
                            x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {t21, ta2_avail, 1, b1}];
                            x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {ta2_avail, tw1, 2, a2}];
                            ra_v1(2,n) = ta2_avail + Cmat(2,n) - tw1;
                            ra_v1(3,n) = Cmat(3,n);
                        else
                            valid_b1_2 = trace_valid_nodes(l, tw, t21, NODES);
                            [~,tb1_2]  = check_resc_occupation(NODES,valid_b1_2,b1,l,V_v1,n,tw1,tw,rs_v1);
                            x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {tb1_2, min(tb1_2+Cmat(1,n),tw1), 1, b1}];
                            x_v1{n}{ni2(n)} = [x_v1{n}{ni2(n)}; {min(tb1_2+Cmat(1,n),tw1), tw1, 2, a2}];
                            ra_v1(1,n) = max(0, tb1_2 + Cmat(1,n) - tw1);
                            ra_v1(2,n) = min(tb1_2 + Cmat(1,n) - tw1 + Cmat(2,n), Cmat(2,n));
                            ra_v1(3,n) = Cmat(3,n);
                        end
                    end
                    next_branches{end+1} = {ra_v1, V_v1, x_v1, pl_v1, rs_v1, tw1}; %#ok<AGROW>
                end
            end
        end
    end
    branches = next_branches;
end

end

%--------------------------------------------------------------------------
% space_variants: generate V_temp variants for n needing space m.
%   - If no one occupies m: single variant (no change).
%   - If n' occupies m with pair_lock(n,n')==n': n must wait (one variant).
%   - If n' occupies m with pair_lock(n,n')==n:  n displaces (one variant).
%   - If n' occupies m with pair_lock(n,n')==0:
%       n' never reset → two variants (wait + displace).
%       n' was reset   → displace only (n gets priority).
%--------------------------------------------------------------------------
function variants = space_variants(V_temp, m, n, pair_lock, reset_since)
    occ = find(V_temp(:, m) > 0);
    occ = occ(occ ~= n);
    if isempty(occ)
        variants = {{V_temp, []}};
        return;
    end
    np       = occ(1);
    lock_val = pair_lock(n, np);
    variants = {};

    if lock_val == 0 || lock_val == np
        variants{end+1} = {V_temp, []};
    end

    if lock_val == n
        V_B      = V_temp;
        V_B(np, m) = 0;
        variants{end+1} = {V_B, np};
    end

    % If lock unknown and np was previously reset, n gets priority (displace only)
    if lock_val == 0 && numel(reset_since) >= np && reset_since(np) > 0
        V_B      = V_temp;
        V_B(np, m) = 0;
        variants{end+1} = {V_B, np};
        variants = variants(end);  % displace only
    end

    if isempty(variants)
        variants = {{V_temp, []}};
    end
end

%--------------------------------------------------------------------------
% relock_Vtemp: record that n lost to the current V_temp occupier of space m.
%--------------------------------------------------------------------------
function pl = relock_Vtemp(pl, V_temp, n, m)
    if m <= 0, return; end
    occ = find(V_temp(:, m) > 0);
    occ = occ(occ ~= n);
    if ~isempty(occ)
        o = occ(1);
        if pl(n,o) == 0 || pl(n,o) == o
            pl(n,o) = o;  pl(o,n) = o;
        end
    end
end
