function [ta_bar_node, ta_bar] = find_node_ta_bar(l, tw, t_avail, NODES)

ta_bar_node = l;
ta_bar = tw;

while ta_bar > t_avail + 1e-5
    parent = NODES{ta_bar_node}{7};   % 7th: parent node

    if isempty(parent) || isnan(parent) || parent <= 0
        return;
    end

    ta_bar_node = parent;
    ta_bar = NODES{ta_bar_node}{5};   % 5th: tw
end

end