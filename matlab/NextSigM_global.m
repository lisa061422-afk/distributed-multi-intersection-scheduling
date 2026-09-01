function [d2, r2, o2, tw1] = NextSigM_global(tw, da, ra, oa, U_temp, const)

N = const.N;
Smax = const.Smax;
BIG_M = const.BIG_M;

r2 = zeros(Smax, N);
o2 = zeros(1, N);
d2 = zeros(1, N);

tol = 1e-8;

% ---------------- compute next significant moment ----------------
if all(U_temp(:) == 0)
    pos_d = da(da > tol & isfinite(da));
    if isempty(pos_d)
        tw1 = tw;
        d2 = da;
        r2 = ra;
        o2 = oa;
        return;
    end
    Lw = min(pos_d);
else
    remain_time = inf(1, N);

    for n = 1:N
        if any(U_temp(n,:))
            [~, m] = find(U_temp(n,:), 1);
            s_temp = U_temp(n,m);
            remain_time(n) = ra(s_temp, n);
        end
    end

    pos_d = da(da > tol & da < BIG_M);
    pos_r = remain_time(remain_time > tol & isfinite(remain_time));

    if isempty(pos_d) && isempty(pos_r)
        tw1 = tw;
        d2 = da;
        r2 = ra;
        o2 = oa;
        return;
    elseif isempty(pos_d)
        Lw = min(pos_r);
    elseif isempty(pos_r)
        Lw = min(pos_d);
    else
        Lw = min(min(pos_d), min(pos_r));
    end
end

tw1 = tw + Lw;
t = Lw;

% ---------------- propagate states from tw to tw1 ----------------
for n = 1:N
    d2(n) = da(n) - t;

    if any(U_temp(n,:))
        [~, m] = find(U_temp(n,:), 1);
        s_temp = U_temp(n,m);

        r2(:,n) = ra(:,n);
        r2(s_temp,n) = ra(s_temp,n) - t;
        o2(n) = oa(n) + t;
    else
        r2(:,n) = ra(:,n);
        o2(n) = oa(n) + sign(sum(ra(:,n))) * t;
    end
end
end