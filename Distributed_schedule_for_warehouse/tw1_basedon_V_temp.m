function tw1 = tw1_basedon_V_temp(V_temp,ra,da,tw,const)


N = const.N;
if all(V_temp(:) == 0)
    Lw = min(da(da>0.00001));
else
    remain_time = zeros(1, N);
    % check each row of V_temp
    for n = 1:N
        if any(V_temp(n, :)) % only handle when this row has nonzero elements
            [~, m] = find(V_temp(n, :), 1); % find the position of nonzero element
            s_temp = V_temp(n, m);
            remain_time(n) = ra(s_temp,n);
        end
    end
    Lw = min(min(da(da>0.00001 & da<1000)),min(remain_time(remain_time>0.00001)));
end
tw1 = tw + Lw;
end