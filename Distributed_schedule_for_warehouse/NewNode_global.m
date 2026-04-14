function NODES_new = NewNode_global(num_nodes, d2, r2, o2, tw1, ni2, ...
    parent_node_index, U_c, U_temp, g, gamma, speed, ra, ra_reset, x, alpha, const)

N = const.N;
Veh = const.Veh;

g_n = zeros(1, N);

% =========================================================
% First update gamma / alpha / g / d2 based on task completion
% =========================================================
for n = 1:N
    % task of vehicle n finishes exactly at tw1
    if sum(ra(:,n)) > 1e-5 && sum(r2(:,n)) <= 1e-5 && ni2(n) >= 1
        task = getTaskProfile(n, ni2(n), const);

        gamma{n}(ni2(n)) = tw1;

        c_total = sum(task.C(task.C > 0));
        start_time = gamma{n}(ni2(n)) - c_total;

        % 用当前 task 自己的 generation time
        g_n(n) = start_time - alpha{n}(ni2(n));

        % reset release timer for next task
        if ni2(n) < const.NI(n)
            d2(n) = const.Dt;
            % fprintf('Task complete: vehicle %d, task %d finished at tw=%.4f, set d2=%g\n', ...
            %     n, ni2(n), tw1, d2(n));
        else
            d2(n) = Inf;
            % fprintf('Task complete: vehicle %d, final task %d finished at tw=%.4f, set d2=Inf\n', ...
            %     n, ni2(n), tw1);
        end
    end
end

g_new = g + sum(g_n);
f_new = g_new;   % heuristic not added yet + 1e-3 * tw1

% =========================================================
% Then pack node AFTER d2/gamma/alpha are finalized
% =========================================================
l = num_nodes + 1;

NODES_new = cell(1,16);
NODES_new{1}  = l;
NODES_new{2}  = d2;
NODES_new{3}  = r2;
NODES_new{4}  = o2;
NODES_new{5}  = tw1;
NODES_new{6}  = ni2;
NODES_new{7}  = parent_node_index;
NODES_new{8}  = U_c;
NODES_new{9}  = U_temp;
NODES_new{10} = g_new;
NODES_new{11} = gamma;
NODES_new{12} = f_new;
NODES_new{13} = speed;
NODES_new{14} = ra_reset;
NODES_new{15} = x;
NODES_new{16} = alpha;

end