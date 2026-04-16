function tw1 = tw1_basedon_U_temp(U_temp,ra,da,tw,const)


N = const.N;
if all(U_temp(:) == 0)
    Lw = min(da(da>0.00001));
else
    remain_time = zeros(1, N);
    % check each row of U_temp
    for n = 1:N
        if any(U_temp(n, :)) % only handle when this row has nonzero elements
            [~, m] = find(U_temp(n, :), 1); % find the position of nonzero element
            s_temp = U_temp(n, m);
            remain_time(n) = ra(s_temp,n);
        end
    end
    Lw = min(min(da(da>0.00001 & da<1000)),min(remain_time(remain_time>0.00001)));
end
tw1 = tw + Lw;
end