function OPEN = prune_nodes_by_ni(NODES, OPEN)
    % 用于从 OPEN 中剪枝：只保留相同 ni 向量中 g_cost 最小的一个

    ni_map = containers.Map();       % key: ni 向量字符串, value: [idx1, idx2, ...]
    g_map = containers.Map();        % key: ni 向量字符串, value: g_costs

    % 构造分组
    for i = 1:length(OPEN)
        idx = OPEN(i);
        r_value = NODES{idx}{3};
        if all(r_value <= 1e-5)      % 满足remain条件才剪枝
            ni = NODES{idx}{6};
            f = NODES{idx}{12}; % f is 12th column 
            key = mat2str(ni);       % 向量编码为字符串 

            if isKey(ni_map, key)
                ni_map(key) = [ni_map(key), idx];
                g_map(key) = [g_map(key), f];
            else
                ni_map(key) = idx;
                g_map(key) = f;
            end
        end
    end

    % 遍历每个 group，只保留 g_cost 最小的节点，其余剪枝
    to_remove = [];
    keys = ni_map.keys;
    for i = 1:length(keys)
        group = ni_map(keys{i});
        g_values = g_map(keys{i});
        if length(group) >= 2
            [~, min_idx] = min(g_values);
            to_remove = [to_remove, group([1:min_idx-1, min_idx+1:end])];
        end
    end

    % 从 OPEN 中移除
    OPEN = setdiff(OPEN, to_remove);
end
