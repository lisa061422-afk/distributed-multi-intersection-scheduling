function [min_f, minIndex] = f_min(NODES,OPEN)
    
    validRows = zeros(length(OPEN), 1);
    validIndices = OPEN(:);  % ensure OPEN is column vector

    % traverse indices of nodes in OPEN，obtain its g_cost
    for i = 1:length(OPEN)
        validRows(i) = NODES{OPEN(i)}{10}; % g_cost is 10th column
    end

    % find the index with minimal g_cost
    [min_f, minindex] = min(validRows);
    minIndex = validIndices(minindex); 
end