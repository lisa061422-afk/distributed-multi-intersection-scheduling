function [NODES,OPEN,LEAF] = expand_array_IN(NODES,OPEN,c_node_index,LEAF,...
    ctx,const)

    N = const.N; M = ctx.M; S = ctx.S; NI_agent = ctx.NI_agent;
    valid_systems= ctx.valid_systems;
    Cmat         = ctx.Cmat;
    MapMat       = ctx.MapMat;      
    ddl          = ctx.ddl;
    arrival_ref  = ctx.arrival_ref;
    if isfield(const,'scheduler_mode') && ~isempty(const.scheduler_mode)
        scheduler_mode = upper(strtrim(const.scheduler_mode));
    else
        scheduler_mode = 'CR-MPC';
    end

    % Priority override: if set, that robot automatically wins any contended column
    if isfield(const, 'priority_n') && ~isempty(const.priority_n)
        priority_n = const.priority_n;
    else
        priority_n = 0;
    end

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
                da(n) = 16.111; %ddl updates to the task period
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
    elseif any(ra(:, valid_systems) > 1e-5, 'all') % have tasks to execute
    % suppress floating-point noise before building U_c
    da = round(da, 6);
    ra = round(ra, 6);
    % only traverse systems that still have unfinished subtasks
    active_systems = valid_systems( any(ra(:, valid_systems) > 1e-5, 1) );

    %-----find which resources all active vehicles need to occupy-------
    U_c = zeros(N,M); 
   for n = active_systems
       s_idx = find(ra(:,n) > 1e-5, 1);
       if ~isempty(s_idx)
          m1 = MapMat(s_idx, n);
          U_c(n, m1) = s_idx;
       end
   end

   %----------------check contention by U_c ---------------------
   col_cnt = sum(U_c(active_systems,:) > 0, 1);

   % if strcmpi(scheduler_mode,'CR-MPC')

       if any(col_cnt > 1) % contention occurs, find all branches
          V_valid = traverse_columns(U_c, priority_n);   % priority_n=0 → normal enumeration
          number_current_node = num_nodes;

          for i = 1:numel(V_valid) % compute each branch
              ra_temp = ra;
              U_temp = V_valid{i};

              [ra_temp2, U_temp2, x2] = resetting_rule(ra,ra_temp,U_temp,U_c,...
                  tw,da,NODES,l,x,ni2,ctx,const); 

              ra_reset = ra_temp2;

              [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,U_temp2,valid_systems,ctx,const); 
              NODES_new = NewNode(number_current_node,d2,r2,o2,tw1,ni2,parent_node_index,...
                  U_c,U_temp2,g,gamma,speed,ra,ra_reset,x2,Cmat,valid_systems,alpha,ddl,arrival_ref,const);

              OPEN = [OPEN, NODES_new{1}];
              number_current_node = number_current_node + 1; 
              NODES = [NODES; {NODES_new}]; 
          end

       else % no contention
           U_temp = U_c; 
           [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const); 
           ra_reset = -1 * ones(S,N);

           NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
               U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref,const);

           OPEN = [OPEN, NODES_new{1}];
           NODES = [NODES; {NODES_new}];
       end

   % elseif strcmpi(scheduler_mode,'FCFS')
   % 
   %     % previous scheduling matrix for non-preemptive continuation
   %     U_prev = zeros(N,M);
   %     if numel(c_node) >= 8 && ~isempty(c_node{8})
   %         U_prev = c_node{8};
   %     end
   % 
   %     if any(col_cnt > 1)
   %         U_temp = build_fcfs_selector(U_c, U_prev, tw, d, active_systems);
   % 
   %         ra_temp = ra;
   %         [ra_temp2, U_temp2, x2] = resetting_rule(ra,ra_temp,U_temp,U_c,...
   %             tw,da,NODES,l,x,ni2,ctx,const);
   % 
   %         ra_reset = ra_temp2;
   % 
   %         [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,U_temp2,valid_systems,ctx,const);
   %         NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
   %             U_c,U_temp2,g,gamma,speed,ra,ra_reset,x2,Cmat,valid_systems,alpha,ddl,arrival_ref,const);
   % 
   %         OPEN = [OPEN, NODES_new{1}];
   %         NODES = [NODES; {NODES_new}];
   % 
   %     else
   %         U_temp = U_c;
   %         [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const);
   %         ra_reset = -1 * ones(S,N);
   % 
   %         NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
   %             U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref,const);
   % 
   %         OPEN = [OPEN, NODES_new{1}];
   %         NODES = [NODES; {NODES_new}];
   %     end
   % 
   % else
   %     error('Unknown const.scheduler_mode = %s. Use ''CR-MPC'' or ''FCFS''.', scheduler_mode);
   % end
end
end

% function U_temp = build_fcfs_selector(U_c, U_prev, tw, d, active_systems)
% 
%     [N,M] = size(U_c);
%     U_temp = zeros(N,M);
% 
%     for m = 1:M
%         cand = find(U_c(:,m) > 0);
%         cand = intersect(cand, active_systems, 'stable');
%         if isempty(cand)
%             continue;
%         end
% 
%         % non-preemptive continuation
%         prev_holder = cand(U_prev(cand,m) > 0);
%         if ~isempty(prev_holder)
%             winner = prev_holder(1);
%             U_temp(winner,m) = U_c(winner,m);
%             continue;
%         end
% 
%         % FCFS: compare actual ready/arrival time to this resource
%         t_ready = inf(numel(cand),1);
%         for k = 1:numel(cand)
%             n = cand(k);
% 
%             % temporary placeholder:
%             % if d(n) means remaining wait until becoming ready,
%             % then current ready time is tw + d(n)
%             t_ready(k) = tw + d(n);
%         end
% 
%         order_table = [t_ready(:), cand(:)];
%         order_table = sortrows(order_table,[1 2]);
% 
%         winner = order_table(1,2);
%         U_temp(winner,m) = U_c(winner,m);
%     end
% end