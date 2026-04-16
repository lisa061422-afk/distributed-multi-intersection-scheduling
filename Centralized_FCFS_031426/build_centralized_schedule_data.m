function DATA = build_centralized_schedule_data(NODES, Path_min, const)
%BUILD_CENTRALIZED_SCHEDULE_DATA
% Recover per-vehicle, per-intersection timing data from centralized nodes.

leaf_idx = Path_min(end);
leaf_node = NODES{leaf_idx};

gamma_all = leaf_node{11};      % gamma_all{n}(k): completion time of k-th visited intersection of vehicle n
alpha_all = leaf_node{16};      % alpha_all{n}(k): stored alpha of k-th visited intersection of vehicle n (if available)

N  = const.N;
P  = const.numInt;
Dt = const.Dt;
Veh = const.Veh;

DATA = struct();
DATA.gamma_all = gamma_all;
DATA.alpha_all_node = alpha_all;

% 按 intersection 存
DATA.alpha_local = cell(P,1);   % alpha_local{p}(n)
DATA.gamma_local = cell(P,1);   % gamma_local{p}(n)
DATA.k_local     = cell(P,1);   % 该车在该 intersection 是第几个 int
DATA.bar_local   = cell(P,1);   % bar_local{p}{n} = [start_t, end_t]
DATA.valid_systems = cell(P,1); % valid_systems{p}

for p = 1:P
    DATA.alpha_local{p} = nan(1,N);
    DATA.gamma_local{p} = nan(1,N);
    DATA.k_local{p}     = nan(1,N);
    DATA.bar_local{p}   = cell(1,N);
    DATA.valid_systems{p} = [];
end

for n = 1:N
    intSeq = Veh(n).intSeq;
    NI = Veh(n).NI;

    if isempty(intSeq)
        continue;
    end

    gamma_n = gamma_all{n};

    % 安全检查
    if numel(gamma_n) < NI
        warning('Vehicle %d: gamma length smaller than NI.', n);
        NI_eff = numel(gamma_n);
    else
        NI_eff = NI;
    end

    for k = 1:NI_eff
        p = intSeq(k);              % 第 k 个访问的 intersection 的编号
        DATA.k_local{p}(n) = k;
        DATA.gamma_local{p}(n) = gamma_n(k);
        DATA.valid_systems{p}(end+1) = n;
    end
end

% ---------- recover alpha_local ----------
for n = 1:N
    intSeq = Veh(n).intSeq;
    NI = Veh(n).NI;

    if isempty(intSeq)
        continue;
    end

    for k = 1:NI
        p = intSeq(k);

        if k == 1
            % 第一个 intersection
            alpha_k = Veh(n).alpha0;
        else
            % 后续 intersection
            prev_gamma = gamma_all{n}(k-1);
            alpha_k = prev_gamma + Dt;
        end

        DATA.alpha_local{p}(n) = alpha_k;
    end
end

% ---------- build simple bars ----------
% 先只画 [alpha_local, gamma_local] 这一整段
for p = 1:P
    vs = DATA.valid_systems{p};
    for ii = 1:numel(vs)
        n = vs(ii);
        a = DATA.alpha_local{p}(n);
        g = DATA.gamma_local{p}(n);

        if ~isnan(a) && ~isnan(g)
            DATA.bar_local{p}{n} = [a, g];
        else
            DATA.bar_local{p}{n} = [];
        end
    end
end

end