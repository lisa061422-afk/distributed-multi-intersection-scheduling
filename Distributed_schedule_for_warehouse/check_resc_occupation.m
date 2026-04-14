function [last_valid_idx,t_avail] = check_resc_occupation(NODES, valid_nodes, resc1,l,V_temp,n,tw1,tw)

% last_valid_idx = -1; t_avail = tw1; %找不到时自动返回tw1
% for i = 1:length(valid_nodes)
%     idx = valid_nodes(i);
%     if idx == l % 该点对应tw当前时刻
%         V = V_temp;
%     else
%         V = NODES{idx}{9}; % 9-th is V
%     end
%     if all(V([1:n-1, n+1:end],resc1) == 0)% resc1未被除了当前系统n以外的占用
%         last_valid_idx = idx; 
%         t_avail = NODES{last_valid_idx}{5}; 
%     else
%         break;
%     end
% end

last_valid_idx = -1; t_avail = tw1; %找不到时自动返回tw1
% 检查 V_temp
if all(V_temp([1:n-1, n+1:end], resc1) == 0)
    last_valid_idx = l; % tw对应的节点index 就是l吧
    t_avail = tw;

    for i = 1:length(valid_nodes)-1
        idx = valid_nodes(i);
        V = NODES{idx}{9}; % 9-th is V
        if all(V([1:n-1, n+1:end],resc1) == 0)% resc1未被除了当前系统n以外的占用
            last_valid_idx = idx; 
            % t_avail = NODES{last_valid_idx}{5}; 
            parent = NODES{last_valid_idx}{7}; 
            t_avail = NODES{parent}{5};
        else
            break;
        end
    end

end
