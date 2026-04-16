function [x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N)

x_prev = cell(1,9);
y_prev = cell(1,8);

% all NaN
for i = 1:9
    x_prev{i} = cell(1,N);
    [x_prev{i}{:}] = deal(NaN);
end
for i = 1:8
    y_prev{i} = cell(1,N);
    [y_prev{i}{:}] = deal(NaN);
end

% initialize x_prev, y_prev using earliest no-delay times
for n = 1:N
    kn = 1;
    chain = pathInfo_agent_chain{n}{kn};   % [int, road, int, ..., 9]
    cseq  = pathInfo_c{n}{kn};             % durations at intersections
    t_alpha = alpha_tilde{n}(kn);

    passed_int_time  = 0;
    passed_road_time = 0;
    int_count = 0;

    for posi = 1:length(chain)
        ag = chain(posi);

        if ag == 9
            % terminal arrival time
            t_terminal = t_alpha + passed_int_time + passed_road_time;
            x_prev{9}{n} = t_terminal;
            continue;
        end

        if mod(posi,2) == 1
            % intersection
            int_count = int_count + 1;

            if int_count > numel(cseq)
                error('Vehicle %d: int_count=%d exceeds numel(pathInfo_c{n}{1})=%d', ...
                    n, int_count, numel(cseq));
            end

            t_arr = t_alpha + passed_int_time + passed_road_time;
            t_dep = t_arr + cseq(int_count);

            x_prev{ag}{n} = t_arr;  
            y_prev{ag}{n} = t_dep;

            passed_int_time = passed_int_time + cseq(int_count);

        else
            % road
            t_road_start = t_alpha + passed_int_time + passed_road_time;
            t_road_end   = t_road_start + Dt;

            x_prev{ag}{n} = t_road_start;
            y_prev{ag}{n} = t_road_end;

            passed_road_time = passed_road_time + Dt;
        end
    end
end

% make bar copies directly
x_prev_bar = x_prev;
y_prev_bar = y_prev;

end