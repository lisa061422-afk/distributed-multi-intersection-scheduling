function [ra_temp, V_temp,x] = resetting_rule(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,ctx,const)

N = const.N; Cmat = ctx.Cmat; MapMat = ctx.MapMat;
tw1 = tw1_basedon_V_temp(V_temp,ra,da,tw,const); % next sig m based on each branch selection
for n = 1:N
    if any(ra(:,n) > 0.00001) && all(V_temp(n, :) == 0) % vehicle n's task is interrupted
        s_interuptted = V_c(n, V_c(n,:) > 0);%被打断的是第几个subtask
        %------------------sub 1 interrupted-------------------------------
        if s_interuptted == 1 %第一个subtask be interruptted 
            ra_temp(:,n) = Cmat(:,n); 
        %------------------sub 2 interrupted-------------------------------
        elseif s_interuptted == 2 %第二个subtask be interruptted
            a1 = MapMat(1,n); % which space a
            t1 = tw1 - Cmat(1,n); %如果sub1能在tw1完成，那么sub1的开始时刻  
            %找到所有需要被检查是否占用了space a的时刻们
            valid_nodes_s2 = trace_valid_nodes(l, tw, t1, NODES); %最后一个点不需要检查，因为它记录的是该时刻之前的resc allo
            % subtask 1 can begin from which node's tw
            [~,ta] = check_resc_occupation(NODES,valid_nodes_s2,a1,l,V_temp,n,tw1,tw); 
            t_avail = max(ta,tw1 - Cmat(1,n));
            %----------------check x to ensure t_avail--------------------
            t_avail = check_x(t_avail, 1, a1, x, n,ctx,const);
            % for p = 1:N
            %     if p == n || isempty(x{p})
            %         continue;
            %     end
            %     x_info = x{p};
            %     for row = size(x_info,1):-1:1
            %         t_start = x_info{row, 1}; t_end = x_info{row, 2}; resc = x_info{row, 4}; 
            %         if resc == a1
            %             if ~(t_end <= t_avail || t_start >= t_avail + C(1,n))  % 有重叠
            %                 t_avail = t_end; 
            %             end 
            %         end
            %     end
            % end
            %-------------------------------------------------------------
            x{n}{ni2(n)} = [x{n}{ni2(n)}; {t_avail, tw1, 1, a1}];
            ra_temp(1,n) = t_avail + Cmat(1,n) - tw1; % won't < 0 
            ra_temp(2:end,n) = Cmat(2:end,n); 
        %------------------sub 3 interrupted-------------------------------
        elseif s_interuptted == 3 %第三个subtask be interruptted
              b1 = MapMat(1,n); % sub1占用哪个 space b1
              a2 = MapMat(2,n); % sub2占用哪个resource, space a2
              t22 = tw1 - Cmat(2,n); 
              valid_nodes_s3_a2 = trace_valid_nodes(l, tw, t22, NODES);
              [~,ta2] = check_resc_occupation(NODES,valid_nodes_s3_a2,a2,l,V_temp,n,tw1,tw); 
              ta2_avail = max(ta2,t22); 
              %----------------check x to ensure t_avail--------------------
              ta2_avail = check_x(ta2_avail, 2, a2, x, n,ctx,const);
              % for p = 1:N
              %   if p == n || isempty(x{p})
              %       continue;
              %   end
              %   x_info = x{p};
              %   for row = size(x_info,1):-1:1
              %       t_start = x_info{row, 1}; t_end = x_info{row, 2}; resc = x_info{row, 4}; 
              %       if resc == a2
              %           if ~(t_end <= ta2_avail || t_start >= ta2_avail + C(2,n))  % 有重叠
              %               ta2_avail = t_end; 
              %           end 
              %       end
              %   end
              % end
            %-------------------------------------------------------------
              t21 = ta2_avail - Cmat(1,n); 
              [ta_bar_node,ta_bar] = find_node_ta_bar(l, tw, ta2_avail, NODES); %find <t2的最大sig m, ta_bar 
              valid_nodes_s3_b1 = trace_valid_nodes(ta_bar_node, ta_bar, t21, NODES); 
              [~,tb1] = check_resc_occupation(NODES,valid_nodes_s3_b1,b1,l,V_temp,n,tw1,tw); 
             %----------------check x to ensure t_avail--------------------
             tb1 = check_x(tb1, 1, b1, x, n,ctx,const);
              % for p = 1:N
              %   if p == n || isempty(x{p})
              %       continue;
              %   end
              %   for k = 1:length(x{p})
              %       x_info = x{p}{k};
              %       if isempty(x_info)
              %           continue;
              %       end
              %       for row = size(x_info,1):-1:1
              %           t_start = x_info{row, 1}; 
              %           t_end = x_info{row, 2}; 
              %           resc = x_info{row, 4}; 
              %           if resc == b1
              %               if ~(t_end <= tb1 || t_start >= tb1 + C(1,n))  % 有重叠
              %                   tb1 = t_end; 
              %               end 
              %           end
              %       end
              %   end
              % end
            %-------------------------------------------------------------
              if tb1 <= t21 + 1e-5 % (15) holds
                  x{n}{ni2(n)} = [x{n}{ni2(n)}; {t21, ta2_avail, 1, b1}]; % ta2_avail won't exceed tw1 in this case
                  x{n}{ni2(n)} = [x{n}{ni2(n)}; {ta2_avail, tw1, 2, a2}];
                  ra_temp(2,n) = ta2_avail + Cmat(2,n) - tw1; % won't < 0
                  ra_temp(3,n) = Cmat(3,n); 
              else % (15) does not hold, then check avail for b1 in [t21, tw]
                  valid_nodes_s3_b1_2 = trace_valid_nodes(l, tw, t21, NODES); 
                  [~,tb1_2] = check_resc_occupation(NODES,valid_nodes_s3_b1_2,b1,l,V_temp,n,tw1,tw); 
                  x{n}{ni2(n)} = [x{n}{ni2(n)}; {tb1_2,min(tb1_2 + Cmat(1,n), tw1), 1, b1}];
                  x{n}{ni2(n)} = [x{n}{ni2(n)}; {min(tb1_2 + Cmat(1,n), tw1),tw1, 2, a2}]; 
                  ra_temp(1,n) = max(0,tb1_2 + Cmat(1,n) - tw1);
                  ra_temp(2,n) = min(tb1_2 + Cmat(1,n) - tw1 + Cmat(2,n), Cmat(2,n));
                  ra_temp(3,n) = Cmat(3,n); 
              end
        end  %保存x时，仍需要更新NODES中节点resource的占用情况，否则后面查不出候补的占用resource,
        % 不能更改已经生成的NODES, 因为存在不同path享有共同节点的情况！
        %改成check_resc_occupation时也要查看迄今为止x中同一个resc的占用情况，因为正向决策变量的取用不会向tw之前看
    end
end



