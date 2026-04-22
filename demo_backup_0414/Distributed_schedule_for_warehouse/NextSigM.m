function [dt,rt,ot,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const,tw1_in)

N = const.N; S = ctx.S;
rt = zeros(S,N); ot = zeros(1,N); dt = zeros(1,N);
%--------------------------compute t_{w+1}---------------------------------
if nargin >= 9 && ~isempty(tw1_in)
    tw1 = tw1_in;
    Lw  = tw1 - tw;
elseif all(U_temp(:) == 0)
    Lw = min(da(da>0.00001));
    tw1 = round(tw + Lw, 6);
else
    remain_time = zeros(1, N);
    for n = valid_systems
        if any(U_temp(n, :))
            [~, m] = find(U_temp(n, :), 1);
            s_temp = U_temp(n, m);
            remain_time(n) = ra(s_temp,n);
        end
    end
    Lw = min(min(da(da>0.00001 & da<1000)),min(remain_time(remain_time>0.00001)));
    tw1 = round(tw + Lw, 6);
end
%-------------------tw to tw+t------------------------------------
t = Lw;
for n = valid_systems
    dt(n) = da(n) - t;
    if any(U_temp(n, :))
        [~, m] = find(U_temp(n, :), 1);
        s_temp = U_temp(n, m);
        rt(:,n) = ra(:,n);
        rt(s_temp,n) = ra(s_temp,n)-t;
        ot(n)=oa(n)+t;
    elseif all(U_temp(n, :) == 0)
        rt(:,n) = ra(:,n);
        ot(n)=oa(n)+sign(sum(ra(:,n)))*t;
    end
end

end
