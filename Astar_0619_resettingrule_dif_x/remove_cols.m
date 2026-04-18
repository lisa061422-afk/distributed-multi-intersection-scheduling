
%remove necessary columns V_c 8, gamma 11, f 12, speed 13

load('nodes_contention.mat');

for i = 1:length(NODES)
    node_data = NODES{i}; 
    %NODES{c_node_index2}([8, 11:13]) = []; 
    node_data(:, [8, 11:13]) = []; 
    NODES{i} = node_data;
end

save('nodes_10col.mat', 'NODES');