function t_avail = check_x(t_avail, s, m, x, n, C, N)

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
                if resc == m && ~(t_end <= t_avail || t_start >= t_avail + C(s,n))
                    t_avail = t_end; % 推迟至当前记录结束时间
                end
            end
        end
    end
