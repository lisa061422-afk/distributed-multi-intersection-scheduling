function valid_nodes = trace_valid_nodes(curr_idx, t_curr, t1, NODES)

valid_nodes = curr_idx;

while t_curr > t1 + 1e-5
    parent_idx = NODES{curr_idx}{7}; %7-th col is parent node
    t_parent = NODES{parent_idx}{5}; %5-th col is tw
    if isempty(parent_idx) || isnan(parent_idx)
       return;
    end
    valid_nodes = [valid_nodes, parent_idx];
    %continue updating to previous' nodes 
    curr_idx = parent_idx; t_curr = t_parent; 
end

% while true
%       parent_idx = NODES{curr_idx}{7}; %7-th col is parent node
%       if isempty(parent_idx) || isnan(parent_idx)
%           return;
%       end
%       valid_nodes = [valid_nodes, parent_idx]; 
%       t_parent = NODES{parent_idx}{5}; %5-th col is tw 
%       if t_parent <= t1 && t_curr > t1 
%          return;
%       else
%           %continue updating to previous' nodes 
%           curr_idx = parent_idx; t_curr = t_parent; 
%       end
% end