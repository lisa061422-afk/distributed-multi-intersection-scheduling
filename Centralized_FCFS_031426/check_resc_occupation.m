function [last_valid_idx, t_avail] = check_resc_occupation(NODES, valid_nodes, resc1, l, U_temp, n, tw1, tw)

last_valid_idx = -1;
t_avail = tw1;   % default: if not found, return tw1

% First check current branch selection U_temp at current tw
if all(U_temp([1:n-1, n+1:end], resc1) == 0)
    last_valid_idx = l;   % current node index
    t_avail = tw;         % resource is available already at current tw

    for i = 1:length(valid_nodes)-1
        idx = valid_nodes(i);

        U = NODES{idx}{9};   % 9th: U_temp stored in that node

        % resource resc1 is not occupied by other systems
        if all(U([1:n-1, n+1:end], resc1) == 0)
            last_valid_idx = idx;

            parent = NODES{last_valid_idx}{7};
            if isempty(parent) || isnan(parent) || parent <= 0
                t_avail = 0;   % reached root
            else
                t_avail = NODES{parent}{5};
            end
        else
            break;
        end
    end
end

end