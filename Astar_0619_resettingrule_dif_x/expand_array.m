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

    pair_lock   = c_node{16};
    reset_since = c_node{17};

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

    % Early termination: all gamma values recorded — no need to wait for period end
    all_gamma_done = true;
    for n_chk = 1:N
        if isempty(gamma{n_chk}) || numel(gamma{n_chk}) < Ni(n_chk)
            all_gamma_done = false;  break;
        end
    end
    if all_gamma_done
        LEAF = [LEAF, l];
        return;
    end

    if all(ra(:) <= 0.00001)
       V_temp = zeros(N,M);
       [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg);
       ra_reset = -1 * ones(S,N);
       new_pair_lock = pair_lock;  % no contention, keep all locks
       NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp,g,gamma,speed,ra,ra_reset,x,cfg,new_pair_lock,reset_since);
       OPEN = [OPEN, NODES_new{1}];
       NODES = [NODES; {NODES_new}];
    elseif any(ra(:) > 0.00001)
       V_c = zeros(N,M);
       active         = any(ra > 0.00001, 1);
       [~, s_idx_vec] = max(ra > 0.00001, [], 1);
       active_n       = find(active);
       s_vals         = s_idx_vec(active_n);
       m1_vals        = Map(sub2ind(size(Map), s_vals, active_n));
       V_c(sub2ind([N, M], active_n, m1_vals)) = s_vals;
       if any(sum(V_c>0) > 1)
          wr_label = 'OFF'; if useWeakRule, wr_label = 'ON'; end
       fprintf('[WR=%s node=%d] contention: ', wr_label, l);
       for dbm=1:M, rows=find(V_c(:,dbm)>0); if numel(rows)>1, fprintf('space%d:N%s ', dbm, mat2str(rows')); end; end
       fprintf('\n  pair_lock:\n'); disp(pair_lock);
       [V_valid, ~, cb_updates] = traverse_columns(V_c, 0, pair_lock, ra);
          number_current_node = num_nodes;
          for i = 1:numel(V_valid)
              ra_temp = ra;
              V_temp = V_valid{i};

              % resetting_rule2 returns a cell array of branches:
              % each branch = {ra_temp2, V_temp2, x2, pl_upd, reset_since2}
              if useWeakRule
                  branches_rr = resetting_rule2(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,cfg,pair_lock,reset_since);
              else
                  branches_rr = resetting_rule2(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,cfg,[],reset_since);
              end

              for bi_rr = 1:numel(branches_rr)
                  br        = branches_rr{bi_rr};
                  ra_temp2  = br{1};  V_temp2      = br{2};
                  x2        = br{3};  pl_upd       = br{4};
                  reset_since2 = br{5};  tw1_rr   = br{6};

                  ra_reset = ra_temp2;

                  % weak rule: update pair_lock for this branch
                  if useWeakRule
                      new_pair_lock = pl_upd;

                      % record winner-loser pair only for genuinely contested spaces
                      cont_m = find(sum(V_c > 0, 1) >= 2);
                      for m_col = cont_m
                          w = find(V_temp2(:, m_col) > 0, 1);
                          if isempty(w), continue; end
                          losers = find(V_c(:, m_col) > 0);
                          losers = losers(losers ~= w);
                          mask   = new_pair_lock(w, losers) == 0 | new_pair_lock(w, losers) == w;
                          valid  = losers(mask);
                          new_pair_lock(w,     valid) = w;
                          new_pair_lock(valid, w    ) = w;
                      end

                      % force-overwrite cycle-break decisions to prevent cycle recurrence
                      cb_mask = cb_updates ~= 0;
                      new_pair_lock(cb_mask) = cb_updates(cb_mask);
                  else
                      new_pair_lock = zeros(N, N);
                  end

                  [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,V_temp2,cfg,tw1_rr);
                  x2 = update_vtemp_x(x2, V_temp2, ni2, r2, tw1, C, N);
                  NODES_new = NewNode(number_current_node,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp2,g,gamma,speed,ra,ra_temp2,x2,cfg,new_pair_lock,reset_since2);
                  OPEN = [OPEN, NODES_new{1}];
                  number_current_node = number_current_node + 1;
                  NODES = [NODES; {NODES_new}];
              end
          end
       elseif all(sum(V_c>0) <= 1)
           V_temp = V_c;
           [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,V_temp,cfg);
           ra_reset = -1 * ones(S,N);
           new_pair_lock = pair_lock;  % no contention, keep locks
           if ~useWeakRule
               new_pair_lock = zeros(N, N);
           end
           x_clean = update_vtemp_x(x, V_temp, ni2, r2, tw1, C, N);
           NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,V_c,V_temp,g,gamma,speed,ra,ra_reset,x_clean,cfg,new_pair_lock,reset_since);
           OPEN = [OPEN, NODES_new{1}];
           NODES = [NODES; {NODES_new}];
       end
    end
end

%--------------------------------------------------------------------------
function x_out = update_vtemp_x(x_in, V_temp, ni2, ra_new, tw_new, C, N)
% 只更新已由resetting_rule2写入过x记录的sub-task时间区间，不创建新记录。
    x_out = x_in;
    for n = 1:N
        if ni2(n) < 1 || ni2(n) > numel(x_out{n}), continue; end
        recs  = x_out{n}{ni2(n)};
        if ~iscell(recs), recs = {}; end   % 初始[]转为cell
        m_idx = find(V_temp(n, :) > 0, 1);

        if ~isempty(m_idx)
            % n在V_temp中：仅当s_cur已有x记录时才更新（避免给未reset的系统画黑框）
            s_cur   = V_temp(n, m_idx);
            has_rec = ~isempty(recs) && any(cellfun(@(r) r{3}==s_cur, num2cell(recs,2)));
            if has_rec
                keep = true(size(recs,1), 1);
                for ri = 1:size(recs,1), keep(ri) = recs{ri,3} ~= s_cur; end
                recs = recs(keep, :);
                t_end = tw_new + ra_new(s_cur, n);
                recs(end+1, :) = {t_end - C(s_cur,n), t_end, s_cur, m_idx};
                x_out{n}{ni2(n)} = recs;
            end
        else
            % n不在V_temp：更新已有ongoing记录的时间
            if isempty(recs), continue; end
            for ri = 1:size(recs, 1)
                s_ri = recs{ri, 3};
                if ra_new(s_ri, n) > 1e-5
                    t_end_new   = tw_new + ra_new(s_ri, n);
                    recs{ri, 1} = t_end_new - C(s_ri, n);
                    recs{ri, 2} = t_end_new;
                end
            end
            x_out{n}{ni2(n)} = recs;
        end
    end
end
