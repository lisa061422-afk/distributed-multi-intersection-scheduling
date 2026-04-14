%% export_demo_images.m
% Load saved ADMM (and optionally FCFS) results, regenerate all demo figures
% and export JS/PNG/FIG files for the HTML viewer.
%
% Run this from the Distributed_schedule_for_warehouse directory.

clear;
% addpath('C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Centralized\Centralized_FCFS_031426');  % 函数已复制到当前目录

%% ── Paths ────────────────────────────────────────────────────────────────
ADMM_MAT  = 'BatchRuns\manual_10r\FourIntersection_ADMM_manual_10r.mat';
FCFS_MAT  = 'BatchRuns\manual_10r\FCFS_manual_10r.mat';
DEMO_DIR  = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';
IMG_DIR   = fullfile(DEMO_DIR, 'images');
CASE_DIR  = 'BatchRuns\manual_10r';

if ~exist(IMG_DIR,  'dir'), mkdir(IMG_DIR);  end

%% ── Load ADMM result ─────────────────────────────────────────────────────
fprintf('Loading ADMM result from: %s\n', ADMM_MAT);
load(ADMM_MAT);
% Variables now in workspace: const, x_prev, y_prev, LocalTreeCache,
%                              residual_r, residual_s, k, delay_costs

N = const.N;

%% ── Macro schedule (Figure 3 from plot_C_ADMM2) ─────────────────────────
fprintf('Plotting macro schedule...\n');

% Capture figure handles before so we can identify the new ones
figs_before = findall(0, 'Type', 'figure');
plot_C_ADMM2(residual_r, residual_s, delay_costs, ...
    x_prev, y_prev, const.pathInfo_agent_chain, N, k);
drawnow;
figs_after = findall(0, 'Type', 'figure');

% plot_C_ADMM2 creates 4 figures in order:
%   1=residuals, 2=delay cost, 3=intersection schedule, 4=road occupancy
% We want Figure 3 (intersection schedule). Sort new figures by number.
new_figs = setdiff(figs_after, figs_before);
[~, ord] = sort(arrayfun(@(f) f.Number, new_figs));
new_figs = new_figs(ord);

if numel(new_figs) >= 3
    fig_macro = new_figs(3);
else
    fig_macro = new_figs(end);   % fallback
    warning('Expected 4 new figures from plot_C_ADMM2, got %d.', numel(new_figs));
end

macroBase = fullfile(CASE_DIR, 'macro_schedule_manual_10r');
savefig(fig_macro, [macroBase '.fig']);
exportgraphics(fig_macro, [macroBase '.png'], 'Resolution', 300);
copyfile([macroBase '.png'], fullfile(IMG_DIR, '10r_S1_optimal_macro.png'));
fprintf('  -> macro saved (.fig + .png)\n');

%% ── Local schedule (distributed) ─────────────────────────────────────────
fprintf('Plotting local schedule (distributed)...\n');
fig_local = figure('Color', 'w', 'Position', [50 50 1800 1080]);
panelPos = {
    [0.04 0.53 0.44 0.42],
    [0.52 0.53 0.44 0.42],
    [0.04 0.05 0.44 0.42],
    [0.52 0.05 0.44 0.42]};
for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    if isempty(cache) || ~isstruct(cache), continue; end
    p = uipanel('Parent', fig_local, ...
        'Units', 'normalized', 'Position', panelPos{agent_i}, ...
        'BackgroundColor', [0.97 0.97 0.97], 'BorderType', 'none');
    plot_local_schedule_final_into_panel_1(p, ...
        cache.NODES, cache.Path, agent_i, cache.valid_systems, const, ...
        'x_prev', x_prev, ...
        'gap', 0.004, 'panelColor', 'w', 'axColor', [0.98 0.98 0.98], ...
        'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0);
end
drawnow; pause(0.3);

localBase = fullfile(CASE_DIR, 'local_schedules_manual_10r');
savefig(fig_local, [localBase '.fig']);
exportapp(fig_local, [localBase '.png']);
copyfile([localBase '.png'], fullfile(IMG_DIR, '10r_S1_optimal_local.png'));
fprintf('  -> local saved (.fig + .png)\n');

%% ── Decision tree panel (reference PNG) ─────────────────────────────────
fprintf('Plotting decision tree panel...\n');
fig_tree = plot_decision_tree_panel(LocalTreeCache);
drawnow;
treeBase = fullfile(CASE_DIR, 'decision_tree_manual_10r');
savefig(fig_tree, [treeBase '.fig']);
exportgraphics(fig_tree, [treeBase '.png'], 'Resolution', 150);
fprintf('  -> tree saved (.fig + .png)\n');

%% ── Export tree JSON for HTML renderer ───────────────────────────────────
fprintf('Exporting tree JSON...\n');
treeJsFile = fullfile(DEMO_DIR, 'tree_10r_S1_optimal.js');
export_tree_json(LocalTreeCache, '10r', 'S1', treeJsFile);

%% ── Export ADMM demo JS ──────────────────────────────────────────────────
fprintf('Exporting ADMM demo JS...\n');
export_demo_json(const, x_prev, y_prev, ...
    '10r · Warehouse', ...
    fullfile(DEMO_DIR, '10r_S1_optimal.js'), ...
    'optimal');

%% ── FCFS ─────────────────────────────────────────────────────────────────
if exist(FCFS_MAT, 'file')
    fprintf('Loading FCFS result from: %s\n', FCFS_MAT);
    load(FCFS_MAT);   % loads: const_fcfs, DATA_fcfs, Cost_fcfs

    fprintf('Plotting FCFS local schedule...\n');
    fig_fcfs = figure('Color', 'w', 'Position', [50 50 1800 1080]);
    for agent_i = 1:4
        vs_fcfs = DATA_fcfs.valid_systems{agent_i};
        if isempty(vs_fcfs), continue; end
        pf = uipanel('Parent', fig_fcfs, ...
            'Units', 'normalized', 'Position', panelPos{agent_i}, ...
            'BackgroundColor', [0.97 0.97 0.97], 'BorderType', 'none');
        plot_local_schedule_final_into_panel_centralized(pf, DATA_fcfs, const_fcfs, ...
            agent_i, vs_fcfs, ...
            'gap', 0.004, 'panelColor', 'w', 'axColor', [0.98 0.98 0.98], ...
            'marg_w', 0.12, 'marg_h', 0.08, 'title_pad', 0);
    end
    drawnow; pause(0.3);

    fcfsBase = fullfile(CASE_DIR, 'fcfs_schedule_manual_10r');
    savefig(fig_fcfs, [fcfsBase '.fig']);
    exportapp(fig_fcfs, [fcfsBase '.png']);
    copyfile([fcfsBase '.png'], fullfile(IMG_DIR, '10r_S1_fcfs_local.png'));
    fprintf('  -> FCFS local saved (.fig + .png)\n');

    fprintf('Exporting FCFS demo JS...\n');
    export_demo_json_fcfs(const_fcfs, DATA_fcfs, ...
        '10r · Warehouse', ...
        fullfile(DEMO_DIR, '10r_S1_fcfs.js'), ...
        'fcfs');
else
    fprintf('[skip] FCFS MAT not found: %s\n', FCFS_MAT);
end

%% ── Done ─────────────────────────────────────────────────────────────────
fprintf('\n=== All exports done ===\n');
fprintf('Images:   %s\n', IMG_DIR);
fprintf('JS files: %s\n', DEMO_DIR);
fprintf('FIG/PNG:  %s\n', CASE_DIR);
