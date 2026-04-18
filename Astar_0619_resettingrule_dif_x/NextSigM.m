function [dt,rt,ot,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg)
N = cfg.N;  S = cfg.S;
rt = zeros(S,N); ot = zeros(1,N); dt = zeros(1,N);
%--------------------------compute t_{w+1}---------------------------------
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
%-------------------tw to tw+t------------------------------------
t = Lw; 
for n = 1:N
    dt(n) = da(n) - t;
    if any(V_temp(n, :))
        [~, m] = find(V_temp(n, :), 1); % find the position of nonzero element
        s_temp = V_temp(n, m);
        rt(:,n) = ra(:,n); %let all remain unchange first, then minus Lw for chosen routes' remain
        rt(s_temp,n) = ra(s_temp,n)-t; 
        ot(n)=oa(n)+t;
    elseif all(V_temp(n, :) == 0)
        rt(:,n) = ra(:,n); 
        ot(n)=oa(n)+sign(sum(ra(:,n)))*t;
    end
end

end

