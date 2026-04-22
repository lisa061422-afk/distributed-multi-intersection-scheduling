% RunRandom.m
% Randomly generates test cases and compares WeakRule ON vs OFF.
% No plots or HTML — outputs a summary table to the console.
%
% ===================== SETTINGS ==============================================
NUM_TRIALS    = 50;       % number of random test cases
N_SYS_RANGE   = [3, 6];  % system count sampled uniformly from this range
D1_MAX        = 3;        % d1 values sampled from U[0, D1_MAX]; set 0 to force all zeros
RNG_SEED      = 123;      % fixed seed for reproducibility; set [] for random each run

cfg_base.v_max        = 20;
cfg_base.detect_range = 510;
cfg_base.W            = 30;
cfg_base.T_period     = 20;
cfg_base.pruneNodes   = false;
cfg_base.useBnB       = false;
cfg_base.useTBound    = true;
cfg_base.timeout_s    = 30;   % per-run timeout (s); 0 = no limit
% =============================================================================

if ~isempty(RNG_SEED), rng(RNG_SEED); end

intCfg = makeIntersectionConfig();

% Pre-allocate result table columns
trialIdx   = zeros(NUM_TRIALS, 1);
nSys       = zeros(NUM_TRIALS, 1);
costON     = nan(NUM_TRIALS, 1);
costOFF    = nan(NUM_TRIALS, 1);
nodesON    = zeros(NUM_TRIALS, 1);
nodesOFF   = zeros(NUM_TRIALS, 1);
leavesON   = zeros(NUM_TRIALS, 1);
leavesOFF  = zeros(NUM_TRIALS, 1);
timeON     = zeros(NUM_TRIALS, 1);
timeOFF    = zeros(NUM_TRIALS, 1);
toON       = false(NUM_TRIALS, 1);
toOFF      = false(NUM_TRIALS, 1);
routeStrs  = cell(NUM_TRIALS, 1);
d1Strs     = cell(NUM_TRIALS, 1);

fprintf('\nRunRandom: %d trials, N_sys in [%d,%d], d1 in [0,%.1f], seed=%s\n', ...
    NUM_TRIALS, N_SYS_RANGE(1), N_SYS_RANGE(2), D1_MAX, num2str(RNG_SEED));
fprintf(repmat('-',1,80)); fprintf('\n');

for k = 1:NUM_TRIALS
    % --- Random config generation -------------------------------------------
    N = randi([N_SYS_RANGE(1), N_SYS_RANGE(2)]);
    routes = randi([1, 12], 1, N);
    if D1_MAX > 0
        d1 = rand(1, N) * D1_MAX;   % uniform [0, D1_MAX]
    else
        d1 = zeros(1, N);
    end

    trialIdx(k)  = k;
    nSys(k)      = N;
    routeStrs{k} = mat2str(routes);
    d1Strs{k}    = ['[' sprintf('%.2f ', d1) ']'];

    fprintf('Trial %3d/%d  N=%d  routes=%s\n', k, NUM_TRIALS, N, mat2str(routes));

    % --- Run WeakRule ON then OFF --------------------------------------------
    for pass = 1:2
        useWeak = (pass == 1);

        cfg = cfg_base;
        cfg.useWeakRule     = useWeak;
        cfg.routeAssignment = routes;
        cfg.d1              = d1;
        cfg.N               = N;
        cfg.M               = intCfg.M;
        cfg.S               = intCfg.S;
        cfg.Map             = intCfg.Map(:, routes);
        cfg.C               = intCfg.C(:,   routes);
        cfg.T               = repmat({{cfg.T_period}}, N, 1);
        cfg.Ni              = ones(1, N);
        cfg                 = buildCfgInitials(cfg);

        [NODES, LEAF, c_idx, timed_out, elapsed] = runAstar(cfg);

        opt = NaN;
        if c_idx > 0, opt = NODES{c_idx}{10}; end

        if useWeak
            costON(k)   = opt;
            nodesON(k)  = size(NODES, 1);
            leavesON(k) = numel(LEAF);
            timeON(k)   = elapsed;
            toON(k)     = timed_out;
        else
            costOFF(k)   = opt;
            nodesOFF(k)  = size(NODES, 1);
            leavesOFF(k) = numel(LEAF);
            timeOFF(k)   = elapsed;
            toOFF(k)     = timed_out;
        end
    end
end

% ===================== Summary table =========================================
fprintf('\n');
fprintf(repmat('=',1,120)); fprintf('\n');
fprintf('%-5s %-4s %-22s %-22s %-9s %-9s %-7s %-8s %-8s %-8s %-8s %-8s %-8s %-6s\n', ...
    'Trial','N','Routes','d1','CostON','CostOFF','Match?', ...
    'NodesON','NodesOFF','LeavON','LeavOFF','TimeON','TimeOFF','Tout?');
fprintf(repmat('-',1,120)); fprintf('\n');

nMatch   = 0;
nDiff    = 0;
nNoPath  = 0;

for k = 1:NUM_TRIALS
    cOn  = costON(k);
    cOff = costOFF(k);

    if isnan(cOn) || isnan(cOff)
        matchStr = 'N/A';
        nNoPath = nNoPath + 1;
    elseif abs(cOn - cOff) < 1e-6
        matchStr = 'YES';
        nMatch = nMatch + 1;
    else
        matchStr = '*** NO';
        nDiff = nDiff + 1;
    end

    toutStr = '';
    if toON(k),  toutStr = [toutStr 'ON ']; end
    if toOFF(k), toutStr = [toutStr 'OFF']; end

    fprintf('%-5d %-4d %-22s %-22s %-9.4f %-9.4f %-7s %-8d %-8d %-8d %-8d %-8.2f %-8.2f %-6s\n', ...
        k, nSys(k), routeStrs{k}, d1Strs{k}, ...
        cOn, cOff, matchStr, ...
        nodesON(k), nodesOFF(k), leavesON(k), leavesOFF(k), ...
        timeON(k), timeOFF(k), toutStr);
end

fprintf(repmat('=',1,120)); fprintf('\n');
fprintf('Trials: %d  |  Cost match: %d  |  Cost DIFFER: %d  |  No-path: %d\n', ...
    NUM_TRIALS, nMatch, nDiff, nNoPath);
fprintf('Avg time ON: %.2fs   Avg time OFF: %.2fs\n', mean(timeON), mean(timeOFF));
fprintf('Total time: %.1fs\n', sum(timeON) + sum(timeOFF));

% ===================== Save CSV ==============================================
matchCol = cell(NUM_TRIALS, 1);
for k = 1:NUM_TRIALS
    if isnan(costON(k)) || isnan(costOFF(k))
        matchCol{k} = 'N/A';
    elseif abs(costON(k) - costOFF(k)) < 1e-6
        matchCol{k} = 'YES';
    else
        matchCol{k} = 'NO';
    end
end

T = table((1:NUM_TRIALS)', nSys, routeStrs, d1Strs, ...
    costON, costOFF, matchCol, ...
    nodesON, nodesOFF, leavesON, leavesOFF, ...
    timeON, timeOFF, toON, toOFF, ...
    'VariableNames', {'Trial','N_sys','Routes','d1', ...
    'Cost_ON','Cost_OFF','CostMatch', ...
    'Nodes_ON','Nodes_OFF','Leaves_ON','Leaves_OFF', ...
    'Time_ON','Time_OFF','Timeout_ON','Timeout_OFF'});

csv_path = fullfile(fileparts(mfilename('fullpath')), ...
    sprintf('random_results_%s.csv', datestr(now,'yyyymmdd_HHMMSS')));
writetable(T, csv_path);
fprintf('Results saved: %s\n', csv_path);
