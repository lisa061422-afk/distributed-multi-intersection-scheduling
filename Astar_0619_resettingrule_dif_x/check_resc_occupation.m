function [last_valid_idx,t_avail] = check_resc_occupation(NODES, valid_nodes, resc1,l,V_temp,n,tw1,tw,reset_since)

if nargin < 9 || isempty(reset_since)
    reset_since = zeros(1, size(V_temp,1));
end

N_sys  = size(V_temp, 1);
others = [1:n-1, n+1:N_sys];

last_valid_idx = -1; t_avail = tw1;
if all(V_temp(others, resc1) == 0)
    last_valid_idx = l;
    t_avail = tw;

    for i = 1:length(valid_nodes)-1
        idx    = valid_nodes(i);
        V      = NODES{idx}{9};
        tw_idx = NODES{idx}{5};

        any_full_occ = false;
        reset_cap    = Inf;
        for j = others
            if V(j, resc1) == 0, continue; end
            if reset_since(j) > 0 && reset_since(j) <= tw_idx
                % j was reset at/before tw_idx — not actually occupying here
                continue;
            elseif reset_since(j) > 0 && reset_since(j) > tw_idx
                % j occupied space but left early at reset_since(j)
                reset_cap = min(reset_cap, reset_since(j));
            else
                any_full_occ = true;
            end
        end

        if ~any_full_occ && isinf(reset_cap)
            % space fully free at this node
            last_valid_idx = idx;
            parent = NODES{last_valid_idx}{7};
            t_avail = NODES{parent}{5};
        elseif ~any_full_occ && isfinite(reset_cap)
            % space was occupied but freed when j was reset
            t_avail = reset_cap;
            break;
        else
            break;
        end
    end

end
