function [ra_temp, V_temp, x, pair_lock] = resetting_rule2(ra,ra_temp,V_temp,V_c,tw,da,NODES,l,x,ni2,cfg,pair_lock)

N = cfg.N;  Map = cfg.Map;  C = cfg.C;

if nargin < 12 || isempty(pair_lock)
    pair_lock = zeros(N, N);
end

tw1 = tw1_basedon_V_temp(V_temp,ra,da,tw,N);
for n = 1:N
    if any(ra(:,n) > 0.00001) && all(V_temp(n, :) == 0) % vehicle n's task is interrupted

        x{n}{ni2(n)} = {};  % discard stale reservations before rescheduling

        s_interuptted = V_c(n, V_c(n,:) > 0);
        %------------------sub 1 interrupted-------------------------------
        if s_interuptted == 1
            ra_temp(:,n) = C(:,n);
            % re-lock from V_temp occupier
            relock_from_Vtemp(Map(1,n));

        %------------------sub 2 interrupted-------------------------------
        elseif s_interuptted == 2
            a1 = Map(1,n);
            t1 = tw1 - C(1,n);
            valid_nodes_s2 = trace_valid_nodes(l, tw, t1, NODES);
            [~,ta] = check_resc_occupation(NODES,valid_nodes_s2,a1,l,V_temp,n,tw1,tw);
            t_avail = max(ta, tw1 - C(1,n));
            [t_avail, blkr] = check_x(t_avail, 1, a1, x, n, C, N);
            % re-lock: from x blocker AND from V_temp occupier
            relock_from_blocker(a1, blkr);
            relock_from_Vtemp(a1);

            x{n}{ni2(n)} = [x{n}{ni2(n)}; {t_avail, tw1, 1, a1}];
            ra_temp(1,n) = t_avail + C(1,n) - tw1;
            ra_temp(2:end,n) = C(2:end,n);

        %------------------sub 3 interrupted-------------------------------
        elseif s_interuptted == 3
            b1 = Map(1,n);
            a2 = Map(2,n);
            t22 = tw1 - C(2,n);
            valid_nodes_s3_a2 = trace_valid_nodes(l, tw, t22, NODES);
            [~,ta2] = check_resc_occupation(NODES,valid_nodes_s3_a2,a2,l,V_temp,n,tw1,tw);
            ta2_avail = max(ta2, t22);
            [ta2_avail, blkr2] = check_x(ta2_avail, 2, a2, x, n, C, N);
            % re-lock for sub2 space
            relock_from_blocker(a2, blkr2);
            relock_from_Vtemp(a2);

            t21 = ta2_avail - C(1,n);
            [ta_bar_node,ta_bar] = find_node_ta_bar(l, tw, ta2_avail, NODES);
            valid_nodes_s3_b1 = trace_valid_nodes(ta_bar_node, ta_bar, t21, NODES);
            [~,tb1] = check_resc_occupation(NODES,valid_nodes_s3_b1,b1,l,V_temp,n,tw1,tw);
            [tb1, blkrb] = check_x(tb1, 1, b1, x, n, C, N);
            % re-lock for sub1 space
            relock_from_blocker(b1, blkrb);
            relock_from_Vtemp(b1);

            if tb1 <= t21 + 1e-5
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {t21, ta2_avail, 1, b1}];
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {ta2_avail, tw1, 2, a2}];
                ra_temp(2,n) = ta2_avail + C(2,n) - tw1;
                ra_temp(3,n) = C(3,n);
            else
                valid_nodes_s3_b1_2 = trace_valid_nodes(l, tw, t21, NODES);
                [~,tb1_2] = check_resc_occupation(NODES,valid_nodes_s3_b1_2,b1,l,V_temp,n,tw1,tw);
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {tb1_2, min(tb1_2+C(1,n),tw1), 1, b1}];
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {min(tb1_2+C(1,n),tw1), tw1, 2, a2}];
                ra_temp(1,n) = max(0, tb1_2 + C(1,n) - tw1);
                ra_temp(2,n) = min(tb1_2 + C(1,n) - tw1 + C(2,n), C(2,n));
                ra_temp(3,n) = C(3,n);
            end
        end
    end
end

    % record occ > n for space m_sp based on x blocker
    function relock_from_blocker(m_sp, blkr)
        if m_sp > 0 && blkr > 0 && blkr ~= n
            if pair_lock(n, blkr) == 0 || pair_lock(n, blkr) == blkr
                pair_lock(n,    blkr) = blkr;
                pair_lock(blkr, n)    = blkr;
            end
        end
    end

    % record occ > n for space m_sp based on current V_temp occupier
    function relock_from_Vtemp(m_sp)
        if m_sp <= 0, return; end
        occ = find(V_temp(:, m_sp) > 0);
        occ = occ(occ ~= n);
        if ~isempty(occ)
            o = occ(1);
            if pair_lock(n, o) == 0 || pair_lock(n, o) == o
                pair_lock(n, o) = o;
                pair_lock(o, n) = o;
            end
        end
    end

end
