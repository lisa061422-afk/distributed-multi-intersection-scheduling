function [x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain_5int(alpha_tilde, pathInfo_agent_chain, pathInfo_c, Dt, N, perturbScale)
% initEarliestXYPrev_fromChain_5int
% Same logic as initEarliestXYPrev_fromChain, adapted for 11-agent topology.
%   Agents 1-5   = intersections
%   Agents 6-10  = roads
%   Agent  11    = terminal
%
% x_prev : cell(1,11)  — arrival times at each agent for each vehicle
% y_prev : cell(1,10)  — departure/exit times (terminal has no y)

NUM_AGENTS       = 11;
NUM_NON_TERMINAL = 10;
TERMINAL         = NUM_AGENTS;

if nargin < 6 || isempty(perturbScale)
    perturbScale = 0;
end

x_prev = cell(1, NUM_AGENTS);
y_prev = cell(1, NUM_NON_TERMINAL);

for i = 1:NUM_AGENTS
    x_prev{i} = cell(1, N);
    [x_prev{i}{:}] = deal(NaN);
end
for i = 1:NUM_NON_TERMINAL
    y_prev{i} = cell(1, N);
    [y_prev{i}{:}] = deal(NaN);
end

for n = 1:N
    kn    = 1;
    chain = pathInfo_agent_chain{n}{kn};   % [int, road, int, ..., 11]
    cseq  = pathInfo_c{n}{kn};             % durations at intersections
    t_alpha = alpha_tilde{n}(kn);

    delta = perturbScale * rand();

    passed_int_time  = 0;
    passed_road_time = 0;
    int_count        = 0;

    for posi = 1:length(chain)
        ag = chain(posi);

        if ag == TERMINAL
            t_terminal = t_alpha + passed_int_time + passed_road_time + delta;
            x_prev{TERMINAL}{n} = t_terminal;
            continue;
        end

        if mod(posi, 2) == 1
            % intersection (odd position in chain)
            int_count = int_count + 1;
            if int_count > numel(cseq)
                error('Vehicle %d: int_count=%d exceeds numel(pathInfo_c{n}{1})=%d', ...
                    n, int_count, numel(cseq));
            end
            t_arr = t_alpha + passed_int_time + passed_road_time + delta;
            t_dep = t_arr + cseq(int_count);

            x_prev{ag}{n} = t_arr;
            y_prev{ag}{n} = t_dep;

            passed_int_time = passed_int_time + cseq(int_count);
        else
            % road (even position in chain)
            t_road_start = t_alpha + passed_int_time + passed_road_time + delta;
            t_road_end   = t_road_start + Dt;

            x_prev{ag}{n} = t_road_start;
            y_prev{ag}{n} = t_road_end;

            passed_road_time = passed_road_time + Dt;
        end
    end
end

x_prev_bar = x_prev;
y_prev_bar = y_prev;
end
