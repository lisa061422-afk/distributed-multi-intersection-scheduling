function OPEN = prune_nodes_by_ni(NODES, OPEN)
    % 从 OPEN 中剪枝：只保留相同 ni 向量中 f_cost 最小的一个
    % 条件：该节点所有 remaining time 均接近 0（即已完成当前阶段调度）

    ni_map = containers.Map();       % key: ni 向量字符串, value: [idx1, idx2, ...]
    g_map  = containers.Map();       % key: ni 向量字符串, value: f_costs

    for i = 1:length(OPEN)
        idx     = OPEN(i);
        r_value = NODES{idx}{3};
        if all(r_value <= 1e-5)      % 只对已完成阶段的节点剪枝
            ni  = NODES{idx}{6};
            f   = NODES{idx}{12};    % f = g + h (h=0 here)
            key = mat2str(ni);

            if isKey(ni_map, key)
                ni_map(key) = [ni_map(key), idx];
                g_map(key)  = [g_map(key),  f];
            else
                ni_map(key) = idx;
                g_map(key)  = f;
            end
        end
    end

    % 每组只保留 f_cost 最小的节点，移除其余
    to_remove = [];
    keys = ni_map.keys;
    for i = 1:length(keys)
        group    = ni_map(keys{i});
        g_values = g_map(keys{i});
        if length(group) >= 2
            [~, min_idx] = min(g_values);
            to_remove = [to_remove, group([1:min_idx-1, min_idx+1:end])]; %#ok<AGROW>
        end
    end

    OPEN = setdiff(OPEN, to_remove);
end
