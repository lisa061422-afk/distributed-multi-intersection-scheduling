function [ra_temp, U_temp, x] = resetting_rule_global(ra, ra_temp, U_temp, U_c, tw, da, NODES, l, x, ni2, ctx, const)

N = const.N;
Cvec = ctx.Cvec;
MapVec = ctx.MapVec;

tw1 = tw1_basedon_U_temp(U_temp, ra, da, tw, const);

for n = 1:N
    if any(ra(:,n) > 1e-5) && all(U_temp(n,:) == 0)   % vehicle n's task is interrupted

        idx_pos = find(U_c(n,:) > 0, 1, 'first');
        if isempty(idx_pos)
            continue;
        end
        s_interrupted = U_c(n, idx_pos);

        %------------------ sub 1 interrupted ------------------
        if s_interrupted == 1
            ra_temp(:,n) = 0;
            ra_temp(1:size(Cvec,1), n) = Cvec(:,n);

        %------------------ sub 2 interrupted ------------------
        elseif s_interrupted == 2
            ra_temp(:,n) = 0;

            a1 = MapVec(1,n);
            t1 = tw1 - Cvec(1,n);

            valid_nodes_s2 = trace_valid_nodes(l, tw, t1, NODES);
            [~, ta] = check_resc_occupation(NODES, valid_nodes_s2, a1, l, U_temp, n, tw1, tw);

            t_avail = max(ta, tw1 - Cvec(1,n));
            t_avail = check_x(t_avail, 1, a1, x, n, ni2, const);

            x{n}{ni2(n)} = [x{n}{ni2(n)}; {t_avail, tw1, 1, a1}];

            ra_temp(1,n) = t_avail + Cvec(1,n) - tw1;
            ra_temp(2:end,n) = Cvec(2:end,n);

        %------------------ sub 3 interrupted ------------------
        elseif s_interrupted == 3
            ra_temp(:,n) = 0;

            b1 = MapVec(1,n);
            a2 = MapVec(2,n);

            t22 = tw1 - Cvec(2,n);
            valid_nodes_s3_a2 = trace_valid_nodes(l, tw, t22, NODES);
            [~, ta2] = check_resc_occupation(NODES, valid_nodes_s3_a2, a2, l, U_temp, n, tw1, tw);

            ta2_avail = max(ta2, t22);
            ta2_avail = check_x(ta2_avail, 2, a2, x, n, ni2, const);

            t21 = ta2_avail - Cvec(1,n);

            [ta_bar_node, ta_bar] = find_node_ta_bar(l, tw, ta2_avail, NODES);
            valid_nodes_s3_b1 = trace_valid_nodes(ta_bar_node, ta_bar, t21, NODES);
            [~, tb1] = check_resc_occupation(NODES, valid_nodes_s3_b1, b1, l, U_temp, n, tw1, tw);

            tb1 = check_x(tb1, 1, b1, x, n, ni2, const);

            if tb1 <= t21 + 1e-5
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {t21, ta2_avail, 1, b1}];
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {ta2_avail, tw1, 2, a2}];

                ra_temp(2,n) = ta2_avail + Cvec(2,n) - tw1;
                ra_temp(3,n) = Cvec(3,n);

            else
                valid_nodes_s3_b1_2 = trace_valid_nodes(l, tw, t21, NODES);
                [~, tb1_2] = check_resc_occupation(NODES, valid_nodes_s3_b1_2, b1, l, U_temp, n, tw1, tw);
                %tb1_2 = check_x(tb1_2, 1, b1, x, n, ni2, const);

                x{n}{ni2(n)} = [x{n}{ni2(n)}; {tb1_2, min(tb1_2 + Cvec(1,n), tw1), 1, b1}];
                x{n}{ni2(n)} = [x{n}{ni2(n)}; {min(tb1_2 + Cvec(1,n), tw1), tw1, 2, a2}];

                ra_temp(1,n) = max(0, tb1_2 + Cvec(1,n) - tw1);
                ra_temp(2,n) = min(tb1_2 + Cvec(1,n) - tw1 + Cvec(2,n), Cvec(2,n));
                ra_temp(3,n) = Cvec(3,n);
            end
        end
    end
end
end