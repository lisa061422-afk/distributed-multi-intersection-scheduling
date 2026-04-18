
function to_remove= Merge(result, NODES, OPEN)
    % 初始化一个空集合用于存储要从 OPEN 中删除的元素
    to_remove = [];

    % 遍历 result 中的每个 cell
    for i = 1:length(result)
        if length(result{i}) >= 2 % 检查 cell 中元素个数是否 >= 2
            g_costs = zeros(1, length(result{i})); % 初始化 g_costs 数组

            % 提取每个数字在 NODES 中的 g cost 值
            for j = 1:length(result{i})
                idx = result{i}(j); % 获取当前 result cell 中的数字
                g_costs(j) = NODES{idx}{10}; % 获取 g cost
            end

            % 找到最小 g cost 和对应的索引
            min_value = min(g_costs);
            %min_elements_idx = find(g_costs == min_value);

            % 提取非最小 g cost 对应的索引值
            for j = 1:length(g_costs)
                if g_costs(j) ~= min_value
                    to_remove(end + 1) = result{i}(j); % 将元素添加到 to_remove 中
                end
            end
        end
    end
    % % 从 OPEN 中删除 to_remove 中的元素
    % OPEN = setdiff(OPEN, to_remove);
end
