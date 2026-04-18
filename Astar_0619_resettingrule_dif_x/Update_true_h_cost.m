load('nodes.mat');

num_nodes = length(NODES);
children = cell(1, num_nodes);  % 用于记录每个节点的子节点

 % 遍历所有节点，建立子节点关系
 for i = 1:num_nodes
     parent = NODES{i}{7};  % parent node is in the 7-th col
     if parent > 0  % 如果存在父节点，将其子节点存储
        children{parent} = [children{parent}, i];
     end
 end

for i = num_nodes:-1:1
    c_node_index = NODES{i}{1}; 
    [leaf_nodes, min_cost_leaf, ~] = find_min_g_cost_leaf(NODES, c_node_index,children); 
    %find all leaf nodes of the current node
    f_cost_leafmin = NODES{min_cost_leaf}{12}; g_cost_c_node = NODES{c_node_index}{10}; 
    h_true_cost = f_cost_leafmin - g_cost_c_node; 
    NODES{c_node_index}{14} = h_true_cost; 
    % f_true_cost = g_cost_c_node + h_true_cost;
    % NODES{c_node_index}{15} = f_true_cost;
end

save('nodes.mat', 'NODES');
