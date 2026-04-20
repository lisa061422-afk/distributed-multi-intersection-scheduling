function [NODES,OPEN,LEAF] = expand_array(NODES,OPEN,c_node_index,useWeakRule,cfg,LEAF)
    N=cfg.N; M=cfg.M; S=cfg.S; Ni=cfg.Ni; T=cfg.T; C=cfg.C; Map=cfg.Map;
    da = zeros(1,N); ra = zeros(S,N); oa = zeros(1,N); ni2 = zeros(1,N); V_c = zeros(N,M);
    num_nodes = size(NODES,1); %the number of all nodes up to now
    %--------------extract current node information-----------------
    c_node = NODES{c_node_index};
    l = c_node{1};
    d = c_node{2}; r = c_node{3}; o = c_node{4};
    tw = c_node{5}; ni = c_node{6};
    g = c_node{10}; gamma = c_node{11}; speed = c_node{13}; ra_reset = c_node{14};
    x = c_node{15};

    % pair_lock: N×N matrix. pair_lock(i,j) = winner means i and j have
    % competed before and winner won; 0 = never competed.
    if numel(c_node) >= 16 && ~isempty(c_node{16})
        pair_lock = c_node{16};
        if ~isequal(size(pair_lock), [N, N])
            pair_lock = zeros(N, N);   % convert old 1×M format on first run
        end
    else
        pair_lock = zeros(N, N);
    end

    parent_node_index = l;
    OPEN(OPEN == parent_node_index) = [];

    %--------------------------from tw- to tw-----------------------
    for n = 1:N
        if d(n) <= 0.00001
            if ni(n) + 1 <= Ni(n)
                ni2(n) = ni(n) + 1;
                da(n) = T{n}{ni2(n)};
                ra(:,n) = C(:,n);
            else
                ni2(n) = Ni(n) + 1;
                da(n) = Inf;
                ra(:,n) = -Inf;
            end
            oa(n) = 0;
        elseif d(n) > 0.00001
            ni2(n) = ni(n);
            da(n) = d(n);
            ra(:,n) = r(:,n);
            oa(n) = o(n);
        end
    end
    %----------------------terminate the node expansion---------------
    if all(ni2 == Ni + 1)
        LEAF = [LEAF, l];
        return;
    end

    if all(ra(:) <= 0.00001)
       V_temp = zeros(N,M);
       [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg);
       ra_reset = -1 * ones(S,N);
       new_pair_lock = pair_lock;  % no contention, keep all locks
       NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp,g,gamma,speed,ra,ra_reset,x,cfg,new_pair_lock);
       OPEN = [OPEN, NODES_new{1}];
       NODES = [NODES; {NODES_new}];
    elseif any(ra(:) > 0.00001)
       V_c = zeros(N,M);
       for n = 1:N
           s_idx = find(ra(:,n) > 0.00001, 1);
           if ~isempty(s_idx)
              m1 = Map(s_idx, n);
              V_c(n, m1) = s_idx;
           end
       end
       if any(sum(V_c>0) > 1)
          [V_valid, ~] = traverse_columns(V_c, 0, pair_lock, ra);
          number_current_node = num_nodes;
          for i = 1:numel(V_valid)
              ra_temp = ra;
              V_temp = V_valid{i};

              % pass pair_lock into resetting rule so interrupt-unlock fires
              if useWeakRule
                  [ra_temp2, V_temp2, x2, pl_upd] = resetting_rule2(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,cfg,pair_lock);
              else
                  [ra_temp2, V_temp2, x2] = resetting_rule2(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,cfg);
                  pl_upd = zeros(N, N);
              end
              ra_reset = ra_temp2;

              % weak rule: update pair_lock for this branch
              if useWeakRule
                  new_pair_lock = pl_upd;

                  % record winner-loser pair only for genuinely contested spaces
                  for m_col = 1:M
                      contenders = find(V_c(:, m_col) > 0);
                      if numel(contenders) >= 2
                          winner_n = find(V_temp2(:, m_col) > 0);
                          if ~isempty(winner_n)
                              w = winner_n(1);
                              for ci = 1:numel(contenders)
                                  lsr = contenders(ci);
                                  if lsr ~= w
                                      % only write if unset or confirming same winner
                                      if new_pair_lock(w, lsr) == 0 || new_pair_lock(w, lsr) == w
                                          new_pair_lock(w,   lsr) = w;
                                          new_pair_lock(lsr, w)   = w;
                                      end
                                  end
                              end
                          end
                      end
                  end

              else
                  new_pair_lock = zeros(N, N);
              end

              [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,V_temp2,cfg);
              NODES_new = NewNode(number_current_node,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp2,g,gamma,speed,ra,ra_reset,x2,cfg,new_pair_lock);
              OPEN = [OPEN, NODES_new{1}];
              number_current_node = number_current_node + 1;
              NODES = [NODES; {NODES_new}];
          end
       elseif all(sum(V_c>0) <= 1)
           V_temp = V_c;
           [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg);
           ra_reset = -1 * ones(S,N);
           new_pair_lock = pair_lock;  % no contention, keep locks
           if ~useWeakRule
               new_pair_lock = zeros(N, N);
           end
           NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp,g,gamma,speed,ra,ra_reset,x,cfg,new_pair_lock);
           OPEN = [OPEN, NODES_new{1}];
           NODES = [NODES; {NODES_new}];
       end
    end
end
