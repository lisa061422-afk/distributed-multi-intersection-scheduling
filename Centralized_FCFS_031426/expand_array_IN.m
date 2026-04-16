function [NODES,OPEN,LEAF] = expand_array_IN(NODES,OPEN,c_node_index,LEAF,...
    ctx,const)

    N = const.N; M = ctx.M; S = ctx.S; NI_agent = ctx.NI_agent;
    valid_systems= ctx.valid_systems;
    Cmat         = ctx.Cmat;
    MapMat       = ctx.MapMat;      
    ddl          = ctx.ddl;
    arrival_ref  = ctx.arrival_ref;


    da = zeros(1,N); ra =  zeros(S,N); oa =  zeros(1,N); ni2 =  zeros(1,N); U_c = zeros(N,M); 
    num_nodes = size(NODES,1); %the number of all nodes up to now 
    %--------------extract current node information-----------------
    c_node = NODES{c_node_index}; %current node
    l = c_node{1}; 
    d = c_node{2}; r = c_node{3}; o = c_node{4};
    tw = c_node{5}; ni = c_node{6}; 
    g = c_node{10}; gamma = c_node{11}; speed = c_node{13}; ra_reset = c_node{14};
    x = c_node{15}; alpha = c_node{16}; 

    parent_node_index = l; 
    OPEN(OPEN == parent_node_index) = []; 

    %--------------------------from tw- to tw-----------------------
    for n = valid_systems
        if d(n) <= 0.00001 % a new task of tank n is generated at tw
            if ni(n) + 1 <= NI_agent(n)
                ni2(n) = ni(n) + 1; 
                % da(n) = T_est{n}(ni2(n)); %ddl updates to the task period 
                da(n) = 15.111; %ddl updates to the task period 
                ra(:,n) = Cmat(:,n); %reset remaining time
            else
                ni2(n) = NI_agent(n) + 1; % all tasks have been completed for tank n 
                da(n) = Inf; 
                ra(:,n) = -Inf;
            end
            oa(n) = 0;
        elseif d(n) > 0.00001
            ni2(n) = ni(n);
            da(n) = d(n);
            ra(:,n) = r(:,n); %assume no interruption 
            oa(n) = o(n);
        end
    end
    %----------------------terminate the node expansion---------------
    if all( ni2(valid_systems) == NI_agent(valid_systems) + 1 )
       NODES = NODES; LEAF = [LEAF, l]; 
       return;
    end

    if all(ra(:,valid_systems) <= 1e-5, 'all') %all tasks have been completed, waiting for ddl to zero at tw 
       %should update the current node information 
       U_temp = zeros(N,M); 
       [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const);  
       ra_reset = -1 * ones(S,N);
       NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
           U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref,const);
       OPEN = [OPEN, NODES_new{1}]; %add the new produced node to OPEN set 
       NODES = [NODES; {NODES_new}]; 
    elseif  any(ra(:, valid_systems) > 1e-5, 'all') %have tasks to execute
       % 只遍历“这列还有任务”的系统
       active_systems = valid_systems( any(ra(:, valid_systems) > 1e-5, 1) );
       %-----find which resources that all vehicles need to occupy-------
       U_c = zeros(N,M); 
       for n = active_systems
           s_idx = find(ra(:,n) > 0.00001, 1);  % 找到第一个满足条件的 s
           if ~isempty(s_idx)
              m1 = MapMat(s_idx, n);
              U_c(n, m1) = s_idx;
           end
       end
       %----------------check contention by U_c ---------------------
       col_cnt = sum(U_c(active_systems,:) > 0, 1);
       if any(col_cnt > 1) %if it has any column that has more than one elements
          % contention occurs, find all branches
          U_temp1 = zeros(N, M); U_temp_list_index = 1; U_temp_list = {};
          V_valid = traverse_columns(U_c, U_temp1, U_temp_list, U_temp_list_index, M, 1); 
          number_current_node = num_nodes;
          for i = 1:numel(V_valid) %compute each branch 
              ra_temp = ra;
              U_temp = zeros(N,M); %initialize 
              U_temp = V_valid{i}; %specific selector variable for each branch 
              %-----------------resetting rule---------------------------
              [ra_temp2, U_temp2,x2] = resetting_rule(ra,ra_temp,U_temp,U_c,...
                  tw,da,NODES,l,x,ni2,ctx,const); 
              
              ra_reset = ra_temp2;
              %-----------------------------------------------------------
              [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,U_temp2,valid_systems,ctx,const); 
              NODES_new = NewNode(number_current_node,d2,r2,o2,tw1,ni2,parent_node_index,...
                  U_c,U_temp2,g,gamma,speed,ra,ra_reset,x2,Cmat,valid_systems,alpha,ddl,arrival_ref,const);
              OPEN = [OPEN, NODES_new{1}]; %add the new produced node to OPEN set 
              number_current_node = number_current_node + 1; 
              NODES = [NODES; {NODES_new}]; 
          end
       elseif all(col_cnt <= 1) %no contention but has tasks
           U_temp = U_c; 
           [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const); 
           ra_reset = -1 * ones(S,N);
           NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
               U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref,const);
           OPEN = [OPEN, NODES_new{1}]; %add the new produced node to OPEN set 
           NODES = [NODES; {NODES_new}]; 
       end 
    end
end

