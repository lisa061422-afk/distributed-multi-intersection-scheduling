%---------------record optimal path--------------------------
resultNodes = [];
current_node_idx =  30704; %test 
%current_node_idx =  c_node_index; %the maximal cost leaf 
while current_node_idx > 0 
    c_node = NODES{current_node_idx};
    resultNodes = [current_node_idx, resultNodes];
    current_node_idx = c_node{7}; %parent node
end

Time = toc

Path_min = resultNodes
save('nodes.mat', 'NODES');