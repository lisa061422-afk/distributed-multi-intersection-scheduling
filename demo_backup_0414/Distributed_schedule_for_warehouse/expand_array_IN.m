function [NODES,OPEN,LEAF,n_pruned_out] = expand_array_IN(NODES,OPEN,c_node_index,LEAF,...
    ctx,const)
    n_pruned_out = 0;

    N = const.N; M = ctx.M; S = ctx.S; NI_agent = ctx.NI_agent;
    valid_systems = ctx.valid_systems;
    Cmat          = ctx.Cmat;
    MapMat        = ctx.MapMat;
    ddl           = ctx.ddl;
    arrival_ref   = ctx.arrival_ref;

    if isfield(const, 'priority_n') && ~isempty(const.priority_n)
        priority_n = const.priority_n;
    else
        priority_n = 0; 
    end

    da = zeros(1,N); ra = zeros(S,N); oa = zeros(1,N); ni2 = zeros(1,N); U_c = zeros(N,M);
    num_nodes = size(NODES,1);
    %--------------extract current node information-----------------
    c_node = NODES{c_node_index};
    l  = c_node{1};
    d  = c_node{2}; r = c_node{3}; o = c_node{4};
    tw = c_node{5}; ni = c_node{6};
    g  = c_node{10}; gamma = c_node{11}; speed = c_node{13};
    x  = c_node{15}; alpha = c_node{16};

    use_weak_rule = isfield(const,'use_weak_rule') && const.use_weak_rule;

    % {17} = pair_lock (N×N)
    if numel(c_node) >= 17 && ~isempty(c_node{17})
        pair_lock = c_node{17};
    else
        pair_lock = zeros(N, N);
    end

    % {18} = reset_since (1×N)
    if numel(c_node) >= 18 && ~isempty(c_node{18})
        reset_since = c_node{18};
    else
        reset_since = zeros(1, N);
    end

    parent_node_index = l;
    OPEN(OPEN == parent_node_index) = [];

    %--------------------------from tw- to tw-----------------------
    for n = valid_systems
        if d(n) <= 0.00001
            if sum(r(:,n)) > 1e-5
                % deadline fired but task still running (displaced/reset): refresh deadline
                ni2(n)  = ni(n);
                da(n)   = 16.111;
                ra(:,n) = r(:,n);
                oa(n)   = o(n);
            elseif ni(n) + 1 <= NI_agent(n)
                ni2(n) = ni(n) + 1;
                da(n)  = 16.111;
                ra(:,n) = Cmat(:,n);
                oa(n) = 0;
            else
                ni2(n) = NI_agent(n) + 1;
                da(n)  = Inf;
                ra(:,n) = -Inf;
                oa(n) = 0;
            end
        elseif d(n) > 0.00001
            ni2(n)  = ni(n);
            da(n)   = d(n);
            ra(:,n) = r(:,n);
            oa(n)   = o(n);
        end
    end
    %----------------------terminate the node expansion---------------
    if all( ni2(valid_systems) == NI_agent(valid_systems) + 1 )
        LEAF = [LEAF, l];
        return;
    end

    all_gamma_done = true;
    for n_chk = valid_systems
        if isempty(gamma{n_chk}) || numel(gamma{n_chk}) < NI_agent(n_chk)
            all_gamma_done = false; break;
        end
    end
    if all_gamma_done
        LEAF = [LEAF, l];
        return;
    end

    if all(ra(:,valid_systems) <= 1e-5, 'all')
        U_temp   = zeros(N,M);
        [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const);
        ra_reset = -1 * ones(S,N);
        NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
            U_c,U_temp,g,gamma,speed,ra,ra_reset,x,Cmat,valid_systems,alpha,ddl,arrival_ref,const,pair_lock,reset_since);
        OPEN  = [OPEN, NODES_new{1}];
        NODES = [NODES; {NODES_new}];

    elseif any(ra(:, valid_systems) > 1e-5, 'all')
        da = round(da, 6);
        ra = round(ra, 6);
        active_systems = valid_systems( any(ra(:, valid_systems) > 1e-5, 1) );

        U_c = zeros(N,M);
        for n = active_systems
            s_idx = find(ra(:,n) > 1e-5, 1);
            if ~isempty(s_idx)
                m1 = MapMat(s_idx, n);
                U_c(n, m1) = s_idx;
            end
        end

        col_cnt = sum(U_c(active_systems,:) > 0, 1);

        if any(col_cnt > 1)
            [V_valid, n_pruned, cb_updates] = traverse_columns(U_c, priority_n, pair_lock, ra);
            n_pruned_out = n_pruned_out + n_pruned;
            number_current_node = num_nodes;

            for i = 1:numel(V_valid)
                ra_temp = ra;
                U_temp  = V_valid{i};

                branches_rr = resetting_rule(ra,ra_temp,U_temp,U_c,...
                    tw,da,NODES,l,x,ni2,ctx,const,pair_lock,reset_since);

                for bi_rr = 1:numel(branches_rr)
                    br           = branches_rr{bi_rr};
                    ra_temp2     = br{1};  U_temp2      = br{2};
                    x2           = br{3};  pl_upd       = br{4};
                    reset_since2 = br{5};  tw1_rr       = br{6};

                    ra_reset = ra_temp2;

                    if use_weak_rule
                        new_pair_lock = pl_upd;
                        cont_m = find(sum(U_c > 0, 1) >= 2);
                        for m_col = cont_m
                            w = find(U_temp2(:, m_col) > 0, 1);
                            if isempty(w), continue; end
                            losers = find(U_c(:, m_col) > 0);
                            losers = losers(losers ~= w);
                            mask   = new_pair_lock(w, losers) == 0 | new_pair_lock(w, losers) == w;
                            valid_l = losers(mask);
                            new_pair_lock(w,       valid_l) = w;
                            new_pair_lock(valid_l, w      ) = w;
                        end
                        cb_mask = cb_updates ~= 0;
                        new_pair_lock(cb_mask) = cb_updates(cb_mask);
                    else
                        new_pair_lock = zeros(N, N);
                    end

                    [d2,r2,o2,tw1] = NextSigM(tw,da,ra_temp2,oa,U_temp2,valid_systems,ctx,const,tw1_rr);
                    x2 = update_vtemp_x(x2, U_temp2, ni2, r2, tw1, Cmat, N);
                    NODES_new = NewNode(number_current_node,d2,r2,o2,tw1,ni2,parent_node_index,...
                        U_c,U_temp2,g,gamma,speed,ra,ra_reset,x2,Cmat,valid_systems,alpha,ddl,arrival_ref,const,new_pair_lock,reset_since2);
                    OPEN  = [OPEN,  NODES_new{1}]; %#ok<AGROW>
                    number_current_node = number_current_node + 1;
                    NODES = [NODES; {NODES_new}]; %#ok<AGROW>
                end
            end

        else % no contention
            U_temp = U_c;
            [d2,r2,o2,tw1] = NextSigM(tw,da,ra,oa,U_temp,valid_systems,ctx,const);
            ra_reset = -1 * ones(S,N);
            if use_weak_rule
                new_pair_lock = pair_lock;
            else
                new_pair_lock = zeros(N, N);
            end
            x_clean = update_vtemp_x(x, U_temp, ni2, r2, tw1, Cmat, N);
            NODES_new = NewNode(num_nodes,d2,r2,o2,tw1,ni2,parent_node_index,...
                U_c,U_temp,g,gamma,speed,ra,ra_reset,x_clean,Cmat,valid_systems,alpha,ddl,arrival_ref,const,new_pair_lock,reset_since);
            OPEN  = [OPEN, NODES_new{1}];
            NODES = [NODES; {NODES_new}];
        end
    end
end

%--------------------------------------------------------------------------
function x_out = update_vtemp_x(x_in, U_temp, ni2, ra_new, tw_new, Cmat, N)
% Update x records for the current time window based on the chosen U_temp.
% Only updates sub-tasks that already have an x record (written by resetting_rule).
    x_out = x_in;
    for n = 1:N
        if ni2(n) < 1 || ni2(n) > numel(x_out{n}), continue; end
        recs  = x_out{n}{ni2(n)};
        if ~iscell(recs), recs = {}; end
        m_idx = find(U_temp(n, :) > 0, 1);

        if ~isempty(m_idx)
            s_cur   = U_temp(n, m_idx);
            has_rec = ~isempty(recs) && any(cellfun(@(r) r{3}==s_cur, num2cell(recs,2)));
            if has_rec
                keep = true(size(recs,1), 1);
                for ri = 1:size(recs,1), keep(ri) = recs{ri,3} ~= s_cur; end
                recs = recs(keep, :);
                t_end = tw_new + ra_new(s_cur, n);
                recs(end+1, :) = {t_end - Cmat(s_cur,n), t_end, s_cur, m_idx};
                x_out{n}{ni2(n)} = recs;
            end
        else
            if isempty(recs), continue; end
            for ri = 1:size(recs, 1)
                s_ri = recs{ri, 3};
                if ra_new(s_ri, n) > 1e-5
                    t_end_new   = tw_new + ra_new(s_ri, n);
                    recs{ri, 1} = t_end_new - Cmat(s_ri, n);
                    recs{ri, 2} = t_end_new;
                end
            end
            x_out{n}{ni2(n)} = recs;
        end
    end
end
