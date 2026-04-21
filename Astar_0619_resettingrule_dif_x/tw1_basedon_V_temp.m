function tw1 = tw1_basedon_V_temp(V_temp,ra,da,tw,N)

if all(V_temp(:) == 0)
    Lw = min(da(da>0.00001));
else
    remain_time = zeros(1, N);
%     for n = 1:N
%         if any(V_temp(n, :))
%             [~, m] = find(V_temp(n, :), 1);
%             s_temp = V_temp(n, m);
%             remain_time(n) = ra(s_temp, n);
%         end
%     end
    s_vec    = max(V_temp, [], 2);
    active_n = find(s_vec > 0)';
    if ~isempty(active_n)
        s_vals = s_vec(active_n)';
        remain_time(active_n) = ra(sub2ind(size(ra), s_vals, active_n));
    end
    Lw = min(min(da(da>0.00001 & da<1000)),min(remain_time(remain_time>0.00001)));
end
tw1 = tw + Lw;
end