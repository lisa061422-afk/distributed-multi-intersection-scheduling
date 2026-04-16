function [Path_min, Optimal_g_cost, best_leaf_idx] = extract_optimal_path_from_nodes(NODES, LEAF, varargin)
%EXTRACT_OPTIMAL_PATH_FROM_NODES
% Extract optimal path by tracing parent nodes from the best leaf.
%
% Inputs:
%   NODES : node cell array
%   LEAF  : leaf node index list
%
% Name-Value:
%   'best_leaf_idx' : if provided, use this leaf directly
%   'criterion'     : 'g_min' (default) or 'g_max'
%
% Outputs:
%   Path_min        : optimal node path from root to selected leaf
%   Optimal_g_cost  : g cost of selected leaf
%   best_leaf_idx   : selected leaf index

p = inputParser;
p.addParameter('best_leaf_idx', [], @(x) isempty(x) || isscalar(x));
p.addParameter('criterion', 'g_min', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});
opt = p.Results;

if isempty(NODES)
    error('NODES is empty.');
end

% ---------- choose leaf ----------
if ~isempty(opt.best_leaf_idx)
    best_leaf_idx = opt.best_leaf_idx;
else
    if isempty(LEAF)
        error('LEAF is empty, cannot extract optimal path.');
    end

    gvals = nan(size(LEAF));
    for i = 1:numel(LEAF)
        idx = LEAF(i);
        gvals(i) = NODES{idx}{10};
    end

    switch lower(string(opt.criterion))
        case "g_min"
            [~, k] = min(gvals);
        case "g_max"
            [~, k] = max(gvals);
        otherwise
            error('Unknown criterion. Use ''g_min'' or ''g_max''.');
    end

    best_leaf_idx = LEAF(k);
end

% ---------- trace back to root ----------
resultNodes = [];
current_node_idx = best_leaf_idx;

while ~isempty(current_node_idx) && ~isnan(current_node_idx) && current_node_idx > 0
    resultNodes = [current_node_idx, resultNodes]; %#ok<AGROW>
    c_node = NODES{current_node_idx};
    parent_idx = c_node{7};

    if isempty(parent_idx) || (isnumeric(parent_idx) && any(isnan(parent_idx)))
        break;
    end
    current_node_idx = parent_idx;
end

Path_min = resultNodes;
Optimal_g_cost = NODES{best_leaf_idx}{10};

end