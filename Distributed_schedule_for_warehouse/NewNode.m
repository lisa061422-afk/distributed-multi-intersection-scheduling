function NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
    U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref, const)

    N = const.N;

    l = num_nodes + 1; %the number of all nodes
    NODES_new{1} = l; 
    NODES_new{2} = d2;
    NODES_new{3} = r2;
    NODES_new{4} = o2;
    NODES_new{5} = tw1; 
    NODES_new{6} = ni2;
    NODES_new{7} = parent_node_index;
    NODES_new{8} = U_c;
    NODES_new{9} = U_temp; 
    %---------------------update g cost------------------------------------
    g_n = zeros(1,N); 
    for n = valid_systems
       if sum(ra(:,n))>0.00001 && sum(r2(:,n))<=0.00001 % tw1(the next significant moment) is gamma
          gamma{n}(ni2(n)) = tw1; %the time exiting the intersection
          % speed(n,ni2(n))=(0-initial_position(n,ni2(n)))/(gamma{n}(ni2(n))-sum(C(:, n))); %each vehicle's speed
          alpha{n}(ni2(n)) = gamma{n}(ni2(n))-sum(Cmat(:, n)); 
          % ✅ delay：用 ddl（generation time）而不是 alpha_tilde
          g_n(n) = alpha{n}(ni2(n)) - arrival_ref(n); % g_n(n) = 0;
       end
       % if abs(g_n(n)) <= 0.00001 && abs(g_n(n)) > 0
       %     g_n(n) = 0;
       % end
    end
    g_sum = sum(g_n); 
    g_new = g + g_sum; 

    NODES_new{10} = g_new; 
    NODES_new{11} = gamma; 

    f_new = g_new + 0; 

    NODES_new{12} = f_new; 
    NODES_new{13} = speed; 
    NODES_new{14} = ra_reset; 
    NODES_new{15} = x; 
    NODES_new{16} = alpha; 

