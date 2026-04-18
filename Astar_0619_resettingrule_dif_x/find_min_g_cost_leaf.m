% load('nodes.mat');
% 
% [leaf_nodes, min_cost_leaf, min_cost] = find_min_g_cost_leaf1(NODES, 89)

function [leaf_nodes, min_cost_leaf, min_cost] = find_min_g_cost_leaf(NODES, start_node,children)
    % 初始化
    num_nodes = length(NODES);

    % 初始化存储叶节点的数组
    leaf_nodes = [];

    % 调用递归函数从指定的根节点开始搜索所有叶节点
    leaf_nodes = find_leaves(NODES, start_node, children, leaf_nodes);

    % 初始化最小成本和对应的节点
    min_cost = Inf;
    min_cost_leaf = -1;

    % 遍历所有叶节点，查找最小的 g_cost
    for i = 1:length(leaf_nodes)
        leaf_node = leaf_nodes(i);
        g_cost = NODES{leaf_node}{10};  % 假设第10列存储 f_cost 12
        if g_cost < min_cost
            min_cost = g_cost;
            min_cost_leaf = leaf_node;
        end
    end
end

function leaf_nodes = find_leaves(NODES, current_node, children, leaf_nodes)
    % 检查当前节点是否有子节点
    if isempty(children{current_node})
        % 如果没有子节点，当前节点就是叶节点
        leaf_nodes = [leaf_nodes, current_node];
    else
        % 递归遍历子节点
        for child = children{current_node}
            leaf_nodes = find_leaves(NODES, child, children, leaf_nodes);
        end
    end
end
