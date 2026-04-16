function t_avail = check_x(t_avail, s, m, x, n, ni2, const)

N = const.N;

% current task profile of vehicle n
task = getTaskProfile(n, ni2(n), const);
Cvec = task.C(:);

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

            % overlap with [t_avail, t_avail + Cvec(s)]
            if resc == m && ~(t_end <= t_avail || t_start >= t_avail + Cvec(s))
                t_avail = t_end;
            end
        end
    end
end

end