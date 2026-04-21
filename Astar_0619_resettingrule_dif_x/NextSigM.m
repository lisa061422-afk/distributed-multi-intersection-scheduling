function [dt,rt,ot,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg,tw1_in)
N = cfg.N;  S = cfg.S;
rt = zeros(S,N); ot = zeros(1,N); dt = zeros(1,N);
%--------------------------compute t_{w+1}---------------------------------
if nargin >= 7 && ~isempty(tw1_in)
    tw1 = tw1_in;
    Lw  = tw1 - tw;
elseif all(V_temp(:) == 0)
    Lw = min(da(da>0.00001));
    tw1 = tw + Lw;
else
    remain_time = zeros(1, N);
    for n = 1:N
        if any(V_temp(n, :))
            [~, m] = find(V_temp(n, :), 1);
            s_temp = V_temp(n, m);
            remain_time(n) = ra(s_temp,n);
        end
    end
    Lw = min(min(da(da>0.00001 & da<1000)),min(remain_time(remain_time>0.00001)));
    tw1 = tw + Lw;
end
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

