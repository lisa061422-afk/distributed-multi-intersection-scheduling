function valid_nodes = trace_valid_nodes(curr_idx, t_curr, t1, NODES)

valid_nodes = curr_idx;

while t_curr > t1 + 1e-5
    parent_idx = NODES{curr_idx}{7};   % 7th: parent node index

    % 到根节点就停
    if isempty(parent_idx) || isnan(parent_idx) || parent_idx <= 0
        return;
    end

    t_parent = NODES{parent_idx}{5};   % 5th: tw

    valid_nodes = [valid_nodes, parent_idx];

    % continue tracing backward
    curr_idx = parent_idx;
    t_curr   = t_parent;
end

end