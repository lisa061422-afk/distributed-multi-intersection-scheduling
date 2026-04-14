%find the time ta_bar which is the largest sig m smaller than ta2_avail
function [ta_bar_node,ta_bar] = find_node_ta_bar(l, tw, t_avail, NODES)

ta_bar_node = l; ta_bar = tw;

while ta_bar > t_avail + 1e-5
    ta_bar_node = NODES{ta_bar_node}{7}; % parent node
    ta_bar = NODES{ta_bar_node}{5}; %对应时刻tw
    % if isempty(parent_idx) || isnan(parent_idx)
    %    return;
    % end
end
