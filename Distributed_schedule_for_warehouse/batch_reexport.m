%% batch_reexport.m
% Re-export all scenarios to regenerate schedules/images/ PNGs
% (run after label changes like Intersection → Merging Zone)
% Run from: Distributed_schedule_for_warehouse/

addpath('C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Centralized\Centralized_FCFS_031426');

jobs = {
%   caseName                    group   scene       priority_n
    'seed_41201_N_8',           '8r',   'S1',       0;
    'manual_10r',               '10r',  'S1',       0;
    'seed_41202_N_10',          '10r',  'S2',       0;
    'seed_41206_N_10',          '10r',  'S3',       0;
    'seed_41202_N_10_pR7',      '10r',  'S2_p7',    7;
    'seed_41206_N_10_pR2',      '10r',  'S3_p2',    2;
    'seed_41206_N_10_pR9',      '10r',  'S3_p9',    9;
};

for i = 1:size(jobs, 1)
    caseName   = jobs{i,1};
    group      = jobs{i,2};
    scene      = jobs{i,3};
    priority_n = jobs{i,4};
    fprintf('\n=== Exporting %s_%s (priority=%d) ===\n', group, scene, priority_n);
    close all;   % clean slate — prevents stale figure handle accumulation
    try
        export_scenario_to_demo(caseName, group, scene, priority_n);
    catch ME
        fprintf('[ERROR] %s: %s\n', caseName, ME.message);
    end
end

fprintf('\n=== batch_reexport done ===\n');
