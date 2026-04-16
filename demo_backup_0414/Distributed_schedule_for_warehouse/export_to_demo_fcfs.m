% export_to_demo_fcfs.m
% ────────────────────────────────────────────────────────────────────────────
% 将已有的 FCFS 计算结果批量导出为 Traffic Demo 所需的 .js 文件。
%
% 前提：对每个 case 已经运行过 MAIN_Centralized_FCFS.m，
%       它会在对应的 batch 文件夹里保存 FCFS_seed{X}_N{Y}.mat。
%
% 用法：直接运行，表格里有结果的 case 会导出，缺文件的会跳过并提示。
% ────────────────────────────────────────────────────────────────────────────

%% ── 路径配置 ─────────────────────────────────────────────────────────────
DEMO_DIR = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';
DIST_BATCH = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Distributed_schedule_for_warehouse\BatchRuns';  % TODO: 确认 FCFS BatchRuns 路径

%% ── 导出映射表（与 export_to_demo.m 保持一致）─────────────────────────────
% 格式：{ batch_seed, batch_N, scene_num }
EXPORT_MAP = {
% seed      N    S#
  4123,     8,   1;   % 8r_S1  balanced
  4125,     8,   2;   % 8r_S2  balanced
  4127,     8,   3;   % 8r_S3  skewed I1
  4120,     10,  2;   % 10r_S2 balanced  (10r_S1 reserved for fixed CR-MPC demo)
  4122,     10,  3;   % 10r_S3 balanced
  4112,     15,  1;   % 15r_S1 balanced
  4113,     15,  2;   % 15r_S2 balanced
  4114,     15,  3;   % 15r_S3 balanced
  4115,     15,  4;   % 15r_S4 balanced
  4101,     20,  1;   % 20r_S1 balanced
  4102,     20,  2;   % 20r_S2 balanced
  4103,     20,  3;   % 20r_S3 balanced
  4110,     20,  4;   % 20r_S4 balanced
  4111,     20,  5;   % 20r_S5 balanced
  4104,     25,  1;   % 25r_S1 balanced
  4108,     25,  2;   % 25r_S2 balanced
  4109,     25,  3;   % 25r_S3 balanced
% ---- 新增 case 在此追加（与 export_to_demo.m 同步）----
};

%% ── 批量导出 ─────────────────────────────────────────────────────────────
if ~exist(DEMO_DIR, 'dir')
    error('Demo schedules 文件夹不存在:\n  %s', DEMO_DIR);
end

fprintf('\n=== export_to_demo_fcfs: 共 %d 个 case ===\n\n', size(EXPORT_MAP,1));

for i = 1:size(EXPORT_MAP, 1)
    batch_seed = EXPORT_MAP{i,1};
    batch_N    = EXPORT_MAP{i,2};
    scene_num  = EXPORT_MAP{i,3};

    caseName  = sprintf('seed_%d_N_%d', batch_seed, batch_N);
    matFile   = fullfile(DIST_BATCH, caseName, ...
                         sprintf('FCFS_seed%d_N%d.mat', batch_seed, batch_N));

    group_tag    = sprintf('%dr', batch_N);
    scene_tag    = sprintf('S%d', scene_num);
    scenario_str = sprintf('%s · %s', group_tag, scene_tag);
    out_name     = sprintf('%s_%s_fcfs.js', group_tag, scene_tag);
    out_path     = fullfile(DEMO_DIR, out_name);

    fprintf('[%d/%d] %s → %s ... ', i, size(EXPORT_MAP,1), caseName, out_name);

    if ~exist(matFile, 'file')
        fprintf('⚠ 跳过（先运行 MAIN_Centralized_FCFS.m 选择该 case）\n');
        continue;
    end

    load(matFile, 'const_fcfs', 'DATA');
    export_demo_json_fcfs(const_fcfs, DATA, scenario_str, out_path, 'fcfs');
end

fprintf('\n=== 完成 ===\n');
fprintf('\n请在 HTML AVAILABLE map 里取消注释以下行：\n\n');
for i = 1:size(EXPORT_MAP, 1)
    batch_N   = EXPORT_MAP{i,2};
    scene_num = EXPORT_MAP{i,3};
    fprintf("  '%dr_S%d_fcfs': true,\n", batch_N, scene_num);
end
