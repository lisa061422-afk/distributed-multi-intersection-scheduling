function r_val = compute_r(x_prev, y_prev, r_local,const)

% compute_r 计算 ADMM 中的原始残差 r
% 
% 输入：
% - x_prev, y_prev：cell，包含每个 agent 的 x 和 y 变量
% - best_alpha, best_gamma：cell，intersection agents 对应的 alpha 和 gamma 值（cell of cell）
% - pathInfo_agent_chain：cell{n}，每个 route n 的 agent_chain
%
% 输出：
% - r_val：标量，表示原始残差的平方和

pathInfo_agent_chain = const.pathInfo_agent_chain; N = const.N;
    r_val = 0; kn = 1;

    for n = 1:N
        agent_chain = pathInfo_agent_chain{n}{kn};

        % ===== 共识约束残差：x - y =====
        for pos = 2:length(agent_chain)
            prev_ag = agent_chain(pos-1);
            curr_ag = agent_chain(pos);
            r_val = r_val + norm(x_prev{curr_ag}{n}(kn) - y_prev{prev_ag}{n}(kn))^2;
        end
    end
    r_val = r_val + r_local;
end
