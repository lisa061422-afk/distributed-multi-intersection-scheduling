% Initialize a cell array to store nodes with contention
contention_NODES = {};

for i = 1:size(NODES, 1)
    V_c = NODES{i}{8}; % Column 8: V_c
    V = NODES{i}{9};   % Column 9: V
    
    % Check if V_c and V are not equal
    if ~isequal(V_c, V)
        contention_NODES = [contention_NODES; NODES(i, :)]; % Append the entire row of the node
    end
end

% Save the updated NODES variable with only contention nodes
NODES = contention_NODES;
save('nodes_contention.mat', 'NODES');
