function Constraints = addMutualExclusion_SpaceLevel_Nonpreemptive( ...
    Constraints, MapMat, Cmat, x, y, valid_systems, best_gamma, eps_gap, agent_i)

    if nargin < 9 || isempty(agent_i), agent_i = -1; end
% Space-level mutual exclusion cuts (nonpreemptive) for ONE intersection agent i
%
% Variables/meaning (consistent with your notation):
%   n: system (vehicle) index
%   s: subtask index (the s-th occupied space in this intersection)
%   MapMat(s,n): space id used by system n at subtask s, 0 means "no subtask"
%   Cmat(s,n): duration for that subtask (same shape as MapMat)
%   x(n,1): alpha (NOT used here)
%   y(n,1): gamma decision variable (sdpvar), used to build start/end of each space
%   best_gamma: last-iteration decision-tree gamma values (for ordering only)
%               can be numeric vector OR cell, but MUST exist for each valid_system
%   eps_gap: optional small gap to avoid equality (e.g., 1e-4). Use 0 if not needed.

    if nargin < 8 || isempty(eps_gap)
        eps_gap = 0;
    end

    % valid_systems = valid_systems(:)';   % row vector

    % ---------- convert best_gamma into numeric ghat(n) (NO existence checks) ----------
    Ntot = size(MapMat,2);
    ghat = zeros(Ntot,1);

    if iscell(best_gamma)
        for n = valid_systems
            if isempty(best_gamma{n})
            fprintf('[WARNING] gamma is empty for system %d in agent %d\n', n, agent_i);
            continue;
        end
            ghat(n) = best_gamma{n}(1);
        end
    else
        for n = valid_systems
            ghat(n) = best_gamma(n);
        end
    end

    % ---------- find contended spaces (appear >= 2 times among valid_systems) ----------
    all_spaces = [];
    for n = valid_systems
        seq = MapMat(:,n);
        all_spaces = [all_spaces; seq(seq>0)];
    end
    uniq_spaces = unique(all_spaces);

    contended_spaces = [];
    for mm = uniq_spaces(:)'
        if sum(all_spaces == mm) >= 2
            contended_spaces = [contended_spaces; mm];
        end
    end

    % ---------- for each contended space: collect occupancies and add e<=s constraints ----------
    for mm = contended_spaces(:)'

        % occ rows: [n, s] meaning system n occupies space mm at subtask s
        occ = [];
        for n = valid_systems
            s_list = find(MapMat(:,n) == mm);   % your assumption: at most one, but allow list
            for t = 1:numel(s_list)
                occ = [occ; n, s_list(t)];
            end
        end

        K = size(occ,1);
        if K <= 1
            continue;
        end

        % For each occupancy k: build symbolic s_expr{k}, e_expr{k}; numeric s_hat(k) for ordering
        s_expr = cell(K,1);
        e_expr = cell(K,1);
        s_hat  = zeros(K,1);

        for k = 1:K
            n = occ(k,1);
            s = occ(k,2);     % <-- THIS is your subtask index s (1,2,3,...)

            % Determine S_n = number of subtasks for system n in this intersection
            Sn = find(MapMat(:,n) > 0, 1, 'last');  % contiguous, so last nonzero row is Sn

            % tail_sum = sum_{q=s+1..Sn} Cmat(q,n)
            tail_sum = sum(Cmat(s+1:Sn, n));

            % end and start of this space-occupancy interval
            % e_{n,s} = gamma_n - sum_{q>s} c_{n,q}
            % s_{n,s} = e_{n,s} - c_{n,s}
            e_expr{k} = y(n,1) - tail_sum; % 系统index 在这里输入的
            s_expr{k} = e_expr{k} - Cmat(s,n);

            % numeric shadow (for ordering only)
            e_hat = ghat(n) - tail_sum;
            s_hat(k) = e_hat - Cmat(s,n);
        end

        % order by last-iteration start times on this space
        [~, order] = sort(s_hat, 'ascend');

        % add adjacent non-overlap constraints: e(prev) + eps <= s(next)
        for r = 1:K-1
            k1 = order(r);
            k2 = order(r+1);
            Constraints = [Constraints, e_expr{k1} + eps_gap <= s_expr{k2}];
        end
    end
end
