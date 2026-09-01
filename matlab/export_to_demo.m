% export_to_demo.m
% ────────────────────────────────────────────────────────────────────────────
% 将选定的 batch 结果批量导出为 Traffic Demo 所需的 .js 文件。
%
% 用法：
%   直接运行本脚本。每行 EXPORT_MAP 对应一个导出任务。
%   如需新增/删除 case，只改 EXPORT_MAP 表格即可。
%   跳过的 case（N=22 等非整值）不列入表格。
%
% 导出后在 HTML AVAILABLE map（约第1554行）取消注释对应行即可激活。
% ────────────────────────────────────────────────────────────────────────────

%% ── Traffic Demo schedules 文件夹路径 ────────────────────────────────────
paths = renke_project_paths();
DEMO_DIR = paths.demoDir;
caseBase = paths.batchDir;

%% ── 导出映射表 ──────────────────────────────────────────────────────────
% 格式：{ batch_seed, batch_N, scene_num }
% scene_num：同一 robot 数量下第几个场景（决定输出文件名里的 SX）
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
% ---- 新增 case 在此追加 ----
};

%% ── 批量导出 ─────────────────────────────────────────────────────────────
if ~exist(DEMO_DIR, 'dir')
    error('Demo schedules 文件夹不存在:\n  %s\n请确认路径正确。', DEMO_DIR);
end

fprintf('\n=== export_to_demo: 共 %d 个 case ===\n\n', size(EXPORT_MAP,1));

for i = 1:size(EXPORT_MAP, 1)
    batch_seed = EXPORT_MAP{i,1};
    batch_N    = EXPORT_MAP{i,2};
    scene_num  = EXPORT_MAP{i,3};

    caseName  = sprintf('seed_%d_N_%d', batch_seed, batch_N);
    caseDir   = fullfile(caseBase, caseName);
    matFile   = fullfile(caseDir, sprintf('FourIntersection_ADMM_seed%d_N%d.mat', ...
                                          batch_seed, batch_N));

    group_tag    = sprintf('%dr', batch_N);
    scene_tag    = sprintf('S%d', scene_num);
    scenario_str = sprintf('%s · %s', group_tag, scene_tag);
    out_name     = sprintf('%s_%s_optimal.js', group_tag, scene_tag);
    out_path     = fullfile(DEMO_DIR, out_name);

    fprintf('[%d/%d] %s → %s ... ', i, size(EXPORT_MAP,1), caseName, out_name);

    if ~exist(matFile, 'file')
        fprintf('⚠ 跳过（.mat 不存在）\n');
        continue;
    end

    load(matFile, 'const', 'x_prev', 'y_prev');
    export_demo_json(const, x_prev, y_prev, scenario_str, out_path, 'optimal');
end

fprintf('\n=== 完成 ===\n');
fprintf('\n请在 HTML AVAILABLE map（第1554行附近）取消注释以下行：\n\n');
for i = 1:size(EXPORT_MAP, 1)
    batch_N   = EXPORT_MAP{i,2};
    scene_num = EXPORT_MAP{i,3};
    fprintf("  '%dr_S%d_optimal': true,\n", batch_N, scene_num);
end
fprintf('\n（FCFS 导出请用 export_to_demo_fcfs.m）\n');
