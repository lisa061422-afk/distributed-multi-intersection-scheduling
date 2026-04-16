%% export_all_interactive_trees_5int.m
% One-click export of interactive decision tree data for ALL traffic (5-int) scenarios.
% Run from Distributed_schedule_for_warehouse/ directory.

DEMO_DIR = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';

% Map: { caseFolderName, group, scene }
scenarios = {
    '5int_manual_10r_S1',     '10r', 'S1';
    '5int_seed41403_N15_S1',  '15r', 'S1';
    '5int_seed41405_N15_S2',  '15r', 'S2';
    '5int_seed41406_N15_S3',  '15r', 'S3';
    '5int_seed41408_N20_S1',  '20r', 'S1';
    '5int_seed41409_N20_S2',  '20r', 'S2';
};

for i = 1:size(scenarios, 1)
    caseName = scenarios{i, 1};
    group    = scenarios{i, 2};
    scene    = scenarios{i, 3};

    matFile = fullfile('BatchRuns', caseName, ...
        sprintf('5int_ADMM_%s.mat', caseName));

    if ~exist(matFile, 'file')
        fprintf('[SKIP] MAT not found: %s\n', matFile);
        continue;
    end

    % tr_ prefix for traffic tree data keys
    outJs = fullfile(DEMO_DIR, sprintf('interactive_tree_tr_%s_%s.js', group, scene));

    fprintf('\n── %s → tr_%s_%s ──\n', caseName, group, scene);
    export_interactive_tree(matFile, outJs, 5, sprintf('tr_%s', group), scene);
end

fprintf('\n=== All 5-int interactive trees exported ===\n');
