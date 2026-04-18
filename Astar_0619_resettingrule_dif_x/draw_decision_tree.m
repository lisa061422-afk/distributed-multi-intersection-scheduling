% Load the NODES matrix
load('nodes.mat');

% Extract the number of nodes
num_nodes = size(NODES, 1);

% Create arrays to store edges (parent-child relations) and labels
edges = [];
node_labels = strings(num_nodes, 1);

for i = 1:num_nodes
    parent = NODES{i}{7};  % Parent node index
    if parent ~= 0  % If the node has a parent
        edges = [edges; parent, i];  % Add edge (parent -> child)
    end
    % Extract the value from the 10th column to label the node   
    value_g = NODES{i}{10}; value_tw = NODES{i}{5};
    if ~isempty(value_tw)
        node_labels(i) = sprintf('Node %d: %.2f, %.2f', i, value_tw, value_g);  % Format node label
    else
        node_labels(i) = sprintf('Node %d', i);  % Default label if no value
    end
    if i == num_nodes%the last node
        value_g = NODES{i}{10}; value_tw = NODES{i}{5};
        node_labels(i) = sprintf('Node %d: %.2f, %.2f', i, value_tw, value_g);
    end
end

% Create a directed graph from the edges
G = digraph(edges(:,1), edges(:,2));

% Plot the decision tree
figure;
h = plot(G, 'Layout', 'layered', 'NodeLabel', node_labels);

% Optionally, adjust visual appearance (e.g., font size)
h.NodeFontSize = 10;

% Define the optimal path (modify this if needed)
%Path_min = [1 2 6 13 26 53 100 146 320 338 515];  % 最优路径的节点index

% Highlight the optimal path in red
highlight(h, Path_min, 'NodeColor', 'r', 'EdgeColor', 'r', 'LineWidth', 2);