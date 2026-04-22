function [last_valid_idx, t_avail] = check_resc_occupation(NODES, valid_nodes, resc1, l, V_temp, n, tw1, tw, reset_since)
% Find the earliest time resc1 becomes available for system n,
% by scanning historical nodes on the current path.
%
% V_temp is NOT used as reference here — V and x are co-decided together.
% Precise sub-task timing is in x (checked separately by check_x).
% reset_since filters stale V records: if j was displaced, its old V entries
% may no longer reflect actual occupancy.
%
% Checking is a single pass: for each historical node, V and reset_since
% are evaluated together — no separate two-phase scan.

if nargin < 9 || isempty(reset_since)
    reset_since = zeros(1, size(V_temp, 1));
end

N_sys  = size(V_temp, 1);
others = [1:n-1, n+1:N_sys];

last_valid_idx = -1; t_avail = tw1;

% First check current V_temp: if no one else occupies resc1 right now,
% scan backward through historical nodes to find how early n can enter.
if all(V_temp(others, resc1) == 0)
    last_valid_idx = l;
    t_avail = tw;

    for i = 1:length(valid_nodes)-1
        idx    = valid_nodes(i);
        V      = NODES{idx}{9};   % historical space assignment at this node
        tw_idx = NODES{idx}{5};   % time of this node

        any_full_occ = false;
        reset_cap    = Inf;

        for j = others
            if V(j, resc1) == 0, continue; end

            % V(j, resc1) > 0: j was assigned to resc1 at this node.
            % Immediately check reset_since(j) to determine validity.
            if reset_since(j) > 0 && reset_since(j) <= tw_idx
                % j was displaced at or before this node's time:
                % V record is fully stale — treat as unoccupied.
                continue;
            elseif reset_since(j) > 0 && reset_since(j) > tw_idx
                % j was occupying resc1 at tw_idx but left early at reset_since(j).
                % Space is free from reset_since(j) onward.
                % Note: reset_since records the overall task reset time,
                % not per-subtask completion — check_x refines this further.
                reset_cap = min(reset_cap, reset_since(j));
            else
                % j is genuinely occupying resc1 with no reset — space is blocked.
                any_full_occ = true;
            end
        end

        if ~any_full_occ && isinf(reset_cap)
            % No occupancy at this node — can push t_avail even earlier.
            last_valid_idx = idx;
            parent = NODES{last_valid_idx}{7};
            t_avail = NODES{parent}{5};
        elseif ~any_full_occ && isfinite(reset_cap)
            % Occupied but j left early at reset_cap — space free from there.
            t_avail = reset_cap;
            break;
        else
            % Genuine occupancy — cannot go further back.
            break;
        end
    end

end
