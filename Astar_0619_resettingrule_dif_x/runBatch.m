% runBatch.m
% Runs multiple test cases with WeakRule ON and OFF, logs to CSV,
% generates individual HTML demos + one comparison HTML.
%
% ===================== EDIT THIS SECTION =====================================
CASES = {
%  name             routeAssignment   d1
  'C1_stagger',    [2 2 5 4 1],      [0.2 0.5 0.5 0.0 0.0];
  'C2_simult',     [2 2 5 4 1],      [0.0 0.0 0.0 0.0 0.0];
  'C3_4arm',       [1 4 11 8],       [0.0 0.0 0.0 0.0];
  'C4_3way',       [2 5 8],          [0.0 0.0 0.0];
  'C5_2straight',  [2 5],            [0.0 0.0];
};
% Columns: case name | routeAssignment (row vector) | d1 (row vector)

cfg_base.v_max        = 20;
cfg_base.detect_range = 510;
cfg_base.W            = 30;
cfg_base.T_period     = 20;
cfg_base.pruneNodes   = false;
cfg_base.timeout_s    = 60;   % seconds per run (0 = no limit)
% =============================================================================

out_dir = fileparts(mfilename('fullpath'));
csv_path = fullfile(out_dir, 'results_comparison.csv');
intCfg  = makeIntersectionConfig();

nCases  = size(CASES, 1);
results = struct([]);   % accumulate for comparison HTML

for ci = 1:nCases
    caseName       = CASES{ci,1};
    routeAssign    = CASES{ci,2};
    d1_vec         = CASES{ci,3};

    % Validate d1 length matches routeAssignment
    if numel(d1_vec) ~= numel(routeAssign)
        warning('runBatch: case "%s" d1 length mismatch — skipping.', caseName);
        continue;
    end

    for useWeak = [true, false]
        ruleStr = 'ON';  if ~useWeak, ruleStr = 'OFF'; end
        fprintf('\n=== %s  WeakRule=%s ===\n', caseName, ruleStr);

        %% Build cfg
        cfg = cfg_base;
        cfg.useWeakRule     = useWeak;
        cfg.routeAssignment = routeAssign;
        cfg.d1              = d1_vec;
        cfg.N               = numel(routeAssign);
        cfg.M               = intCfg.M;
        cfg.S               = intCfg.S;
        cfg.Map             = intCfg.Map(:, routeAssign);
        cfg.C               = intCfg.C(:,   routeAssign);
        cfg.T               = repmat({{cfg.T_period}}, cfg.N, 1);
        cfg.Ni              = ones(1, cfg.N);
        cfg                 = buildCfgInitials(cfg);

        %% Run A*
        [NODES, LEAF, c_idx, timed_out, elapsed] = runAstar(cfg);

        %% Extract result
        if c_idx > 0
            opt_cost = NODES{c_idx}{10};
        else
            opt_cost = NaN;
        end
        n_leaves = numel(LEAF);
        n_nodes  = size(NODES, 1);

        fprintf('  g=%.4f  leaves=%d  nodes=%d  time=%.1fs%s\n', ...
            opt_cost, n_leaves, n_nodes, elapsed, ...
            repmat(' [TIMEOUT]', 1, timed_out));

        %% Log to CSV
        logResultCSV(cfg, opt_cost, n_leaves, n_nodes, elapsed, timed_out, csv_path);

        %% Save individual HTML demo
        if ~isempty(LEAF)
            html_name = sprintf('demo_%s_%s.html', caseName, ruleStr);
            html_path = fullfile(out_dir, html_name);
            exportHTML(NODES, LEAF, cfg, html_path);
        else
            html_name = '';
        end

        %% Store for comparison
        ri = numel(results) + 1;
        results(ri).caseName  = caseName;
        results(ri).useWeak   = useWeak;
        results(ri).ruleStr   = ruleStr;
        results(ri).routes    = routeAssign;
        results(ri).d1        = d1_vec;
        results(ri).optCost   = opt_cost;
        results(ri).nLeaves   = n_leaves;
        results(ri).nNodes    = n_nodes;
        results(ri).elapsed   = elapsed;
        results(ri).timedOut  = timed_out;
        results(ri).htmlFile  = html_name;
    end
end

%% Generate comparison HTML
comp_path = fullfile(out_dir, 'comparison.html');
exportCompareHTML(results, CASES(:,1), comp_path);

fprintf('\n=== Batch complete ===\n');
fprintf('CSV:         %s\n', csv_path);
fprintf('Comparison:  %s\n', comp_path);
