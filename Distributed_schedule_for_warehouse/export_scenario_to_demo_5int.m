%% export_scenario_to_demo_5int.m
% Export one 5-intersection ADMM scheduling result to the HTML demo.
%
% Usage (run from Distributed_schedule_for_warehouse/):
%   export_scenario_to_demo_5int('5int_seed42005_N15', '15r', 'S1')
%   export_scenario_to_demo_5int('5int_seed42001_N15', '15r', 'S2')
%   export_scenario_to_demo_5int('5int_manual_10r',   '10r', 'S1')
%   export_scenario_to_demo_5int('5int_seed42003_N20', '20r', 'S1')
%
% Arguments:
%   caseName — folder name under BatchRuns/, e.g. '5int_seed42005_N15'
%   group    — HTML group key, e.g. '15r'
%   scene    — HTML scene key, e.g. 'S1' / 'S2'

function export_scenario_to_demo_5int(caseName, group, scene)

CASE_DIR  = fullfile('BatchRuns', caseName);
DEMO_DIR  = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';
if ~exist(DEMO_DIR, 'dir'), mkdir(DEMO_DIR); end

pfx      = sprintf('%s_%s', group, scene);   % e.g. '15r_S1'
ADMM_MAT = fullfile(CASE_DIR, sprintf('5int_ADMM_%s.mat', caseName));
FCFS_MAT = fullfile(CASE_DIR, sprintf('FCFS_%s.mat', caseName));

%% ── Load ADMM ──────────────────────────────────────────────────────────────
if ~exist(ADMM_MAT, 'file')
    error('MAT file not found: %s', ADMM_MAT);
end
fprintf('Loading %s ...\n', ADMM_MAT);
load(ADMM_MAT);   % loads: const, x_prev, y_prev, LocalTreeCache,
                  %        residual_r, residual_s, delay_costs, k

%% ── ADMM demo JS ─────────────────────────────────────────────────────────
fprintf('Exporting ADMM demo JS...\n');
export_demo_json(const, x_prev, y_prev, ...
    sprintf('%s · %s · Optimal', group, scene), ...
    fullfile(DEMO_DIR, sprintf('%s_optimal.js', pfx)), ...
    'optimal', 0);

%% ── FCFS ─────────────────────────────────────────────────────────────────
if exist(FCFS_MAT, 'file')
    fprintf('Loading FCFS: %s\n', FCFS_MAT);
    fcfs_data  = load(FCFS_MAT);   % isolated — won't overwrite workspace
    const_fcfs = fcfs_data.const_fcfs;
    DATA_fcfs  = fcfs_data.DATA_fcfs;
    export_demo_json_fcfs(const_fcfs, DATA_fcfs, ...
        sprintf('%s · %s · FCFS', group, scene), ...
        fullfile(DEMO_DIR, sprintf('%s_fcfs.js', pfx)), 'fcfs');
    fprintf('  -> FCFS done\n');
else
    fprintf('[skip] No FCFS MAT found: %s\n', FCFS_MAT);
end

%% ── Print HTML instructions ───────────────────────────────────────────────
fprintf('\n=== Done: %s → %s ===\n', caseName, pfx);
fprintf('\nAdd/update in warehouse_amr_demo_test.html:\n');
fprintf('  SCENARIO_GROUPS_TR: add ''%s'' group with scene ''%s''\n', group, scene);
fprintf('    ''%s'': { label:''%s Vehicles'', scenes:[''%s''] },\n', group, group, scene);
fprintf('  AVAILABLE_TR:\n');
fprintf('    ''%s_optimal'': true,\n', pfx);
if exist(FCFS_MAT, 'file')
    fprintf('    ''%s_fcfs'':    true,\n', pfx);
end
end
