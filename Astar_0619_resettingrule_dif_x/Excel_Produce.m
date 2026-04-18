% Load the NODES matrix from the .mat file
load('nodes_10col.mat');

% Initialize a cell array to store the node data for Excel
num_nodes = length(NODES);
node_data = {};  % We will dynamically build this cell array

for i = 1:num_nodes
    node = NODES{i};
    row_data = [];  % To store the flattened row data for the current node
    
    % Process each of the 14 fields in the node
    for j = 1:9  % The last column will be the output
        value = node{j};
        
        if isnumeric(value) || islogical(value)
            row_data = [row_data, value(:)'];  % Flatten matrix or vector
        else
            row_data = [row_data, NaN];  % For non-numeric fields, insert NaN
        end
    end
    
    % Append the 14th field (output) as the last column
    output_value = node{10};
    row_data = [row_data, output_value];
    
    % Append this row to the node_data cell array
    node_data = [node_data; row_data];
end

% Convert cell array to table
T = cell2table(node_data);

% Write the table to an Excel file
writetable(T, 'data_Fig11.xlsx');
disp('Nodes data has been saved to nodes_data.xlsx');
