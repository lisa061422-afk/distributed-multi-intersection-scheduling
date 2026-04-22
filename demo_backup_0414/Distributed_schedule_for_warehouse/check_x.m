function [t_avail, blocker] = check_x(t_avail, s, m, x, n, Cmat, N)
% Returns t_avail (possibly pushed forward) and blocker = system that caused
% the last push (0 if no push). Caller uses blocker to record pair_lock.

    blocker = 0;
    for p = 1:N
        if p == n || isempty(x{p})
            continue;
        end
        for k = 1:length(x{p})
            x_info = x{p}{k};
            if isempty(x_info)
                continue;
            end
            for row = size(x_info,1):-1:1
                t_start = x_info{row, 1};
                t_end   = x_info{row, 2};
                resc    = x_info{row, 4};
                if resc == m && ~(t_end <= t_avail || t_start >= t_avail + Cmat(s,n))
                    t_avail = t_end;
                    blocker = p;
                end
            end
        end
    end
end
