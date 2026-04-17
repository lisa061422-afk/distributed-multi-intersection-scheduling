%% export_all_interactive_trees.m
% One-click export of interactive decision tree data for ALL warehouse scenarios.
% Run from Distributed_schedule_for_warehouse/ directory.

DEMO_DIR = 'C:\Users\robin\OneDrive\Documents\Github_file\demo_backup_0414\Traffic_Demo\schedules';

% Map: { caseFolderName, group, scene }
scenarios = {
    'seed_41201_N_8',   '8r',  'S1';
    'manual_10r',       '10r', 'S1';
    'seed_41202_N_10',  '10r', 'S2';
    'seed_41206_N_10',  '10r', 'S3';
    'manual_10r_S5',    '10r', 'S5';
    'manual_10r_S6',    '10r', 'S6';
    'manual_10r_S7',    '10r', 'S7';
    'manual_10r_S8',    '10r', 'S8';
    'seed_41207_N_15',  '15r', 'S1';
    'seed_41218_N_15',  '15r', 'S2';
    'manual_20r',       '20r', 'S1';
};

for i = 1:size(scenarios, 1)
    caseName = scenarios{i, 1};
    group    = scenarios{i, 2};
    scene    = scenarios{i, 3};

    matFile = fullfile('BatchRuns', caseName, ...
        sprintf('FourIntersection_ADMM_%s.mat', caseName));

    if ~exist(matFile, 'file')
        fprintf('[SKIP] MAT not found: %s\n', matFile);
        continue;
    end

    outJs = fullfile(DEMO_DIR, sprintf('interactive_tree_%s_%s.js', group, scene));

    fprintf('\n── %s → %s_%s ──\n', caseName, group, scene);
    export_interactive_tree(matFile, outJs, 4, group, scene);
end

fprintf('\n=== All interactive trees exported ===\n');
fprintf('Files in: %s\n', DEMO_DIR);
