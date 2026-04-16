%% export_scenario_to_demo.m
% Export ONE scenario (or one priority-override variant) to the HTML demo.
%
% Usage (run from Distributed_schedule_for_warehouse/):
%   export_scenario_to_demo('seed_41201_N_8', '8r', 'S1')
%   export_scenario_to_demo('seed_41201_N_8_pR3', '8r', 'S1_p3', 3)
%
% Arguments:
%   caseName   — folder name under BatchRuns/, e.g. 'seed_41201_N_8'
%   group      — HTML group key,  e.g. '8r' / '10r' / '12r'
%   scene      — HTML scene key,  e.g. 'S1' / 'S1_p3'
%   priority_n — (optional, default 0) robot index with highest priority;
%                0 = normal run (no override). When > 0, FCFS export is skipped.

function export_scenario_to_demo(caseName, group, scene, priority_n)

if nargin < 4 || isempty(priority_n), priority_n = 0; end

% addpath('C:\Users\robin\OneDrive\桌面\RES_Spring2026\CODE\Traffic_Centralized\Centralized_FCFS_031426');  % 函数已复制到当前目录

CASE_DIR  = fullfile('BatchRuns', caseName);
DEMO_DIR  = 'C:\Users\rwang26\Downloads\Research_Spring2026\CODE\Renke-project1-main\Traffic_Demo\schedules';
IMG_DIR   = fullfile(DEMO_DIR, 'images');
if ~exist(IMG_DIR, 'dir'), mkdir(IMG_DIR); end

pfx       = sprintf('%s_%s', group, scene);   % e.g. '8r_S1' or '8r_S1_p3'
ADMM_MAT  = fullfile(CASE_DIR, sprintf('FourIntersection_ADMM_%s.mat', caseName));
FCFS_MAT  = fullfile(CASE_DIR, sprintf('FCFS_%s.mat', caseName));

%% ── Load ADMM ────────────────────────────────────────────────────────────
fprintf('Loading %s ...\n', ADMM_MAT);
load(ADMM_MAT);   % const, x_prev, y_prev, LocalTreeCache, residual_r/s, k, delay_costs
N = const.N;

%% ── Macro (Figure 3 from plot_C_ADMM2) ──────────────────────────────────
fprintf('Macro schedule...\n');
figs_before = findall(0, 'Type', 'figure');
plot_C_ADMM2(residual_r, residual_s, delay_costs, ...
    x_prev, y_prev, const.pathInfo_agent_chain, N, k);
drawnow;
figs_after = findall(0, 'Type', 'figure');
new_figs = setdiff(figs_after, figs_before);
[~, ord] = sort(arrayfun(@(f) f.Number, new_figs));
new_figs = new_figs(ord);
fig_macro = new_figs(min(3, end));

macroBase = fullfile(CASE_DIR, sprintf('macro_%s', caseName));
savefig(fig_macro, [macroBase '.fig']);
exportgraphics(fig_macro, [macroBase '.png'], 'Resolution', 300);
copyfile([macroBase '.png'], fullfile(IMG_DIR, sprintf('%s_optimal_macro.png', pfx)));
fprintf('  -> macro done\n');

%% ── Local schedule (distributed) ────────────────────────────────────────
fprintf('Local schedule (distributed)...\n');
fig_local = figure('Color','w','Position',[50 50 1800 1080]);
panelPos = {[0.04 0.53 0.44 0.42],[0.52 0.53 0.44 0.42],[0.04 0.05 0.44 0.42],[0.52 0.05 0.44 0.42]};
for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    if isempty(cache) || ~isstruct(cache), continue; end
    p = uipanel('Parent',fig_local,'Units','normalized','Position',panelPos{agent_i},...
        'BackgroundColor',[0.97 0.97 0.97],'BorderType','none');
    plot_local_schedule_final_into_panel_1(p, cache.NODES, cache.Path, ...
        agent_i, cache.valid_systems, const, ...
        'x_prev', x_prev, 'gap',0.004,'panelColor','w','axColor',[0.98 0.98 0.98],...
        'marg_w',0.12,'marg_h',0.08,'title_pad',0);
end
drawnow; pause(0.3);
localBase = fullfile(CASE_DIR, sprintf('local_%s', caseName));
savefig(fig_local, [localBase '.fig']);
exportapp(fig_local, [localBase '.png']);
copyfile([localBase '.png'], fullfile(IMG_DIR, sprintf('%s_optimal_local.png', pfx)));
close(fig_local);
fprintf('  -> local done\n');

%% ── Decision tree JSON ───────────────────────────────────────────────────
fprintf('Tree JSON...\n');
export_tree_json(LocalTreeCache, group, scene, ...
    fullfile(DEMO_DIR, sprintf('tree_%s_optimal.js', pfx)));

%% ── ADMM demo JS ─────────────────────────────────────────────────────────
fprintf('ADMM demo JS...\n');
export_demo_json(const, x_prev, y_prev, ...
    sprintf('%s · %s · Optimal', group, scene), ...
    fullfile(DEMO_DIR, sprintf('%s_optimal.js', pfx)), ...
    'optimal', priority_n);

%% ── FCFS (only for normal / non-priority runs) ───────────────────────────
if priority_n == 0 && exist(FCFS_MAT, 'file')
    fprintf('Loading FCFS...\n');
    fcfs_data = load(FCFS_MAT);   % isolated load — won't overwrite workspace vars
    const_fcfs = fcfs_data.const_fcfs;
    DATA_fcfs  = fcfs_data.DATA_fcfs;
    fig_fcfs = figure('Color','w','Position',[50 50 1800 1080]);
    for agent_i = 1:4
        vs = DATA_fcfs.valid_systems{agent_i};
        if isempty(vs), continue; end
        pf = uipanel('Parent',fig_fcfs,'Units','normalized','Position',panelPos{agent_i},...
            'BackgroundColor',[0.97 0.97 0.97],'BorderType','none');
        plot_local_schedule_final_into_panel_centralized(pf, DATA_fcfs, const_fcfs, ...
            agent_i, vs, 'gap',0.004,'panelColor','w','axColor',[0.98 0.98 0.98],...
            'marg_w',0.12,'marg_h',0.08,'title_pad',0);
    end
    drawnow; pause(0.3);
    fcfsBase = fullfile(CASE_DIR, sprintf('fcfs_%s', caseName));
    savefig(fig_fcfs, [fcfsBase '.fig']);
    exportapp(fig_fcfs, [fcfsBase '.png']);
    copyfile([fcfsBase '.png'], fullfile(IMG_DIR, sprintf('%s_fcfs_local.png', pfx)));
    close(fig_fcfs);
    export_demo_json_fcfs(const_fcfs, DATA_fcfs, ...
        sprintf('%s · %s · FCFS', group, scene), ...
        fullfile(DEMO_DIR, sprintf('%s_fcfs.js', pfx)), 'fcfs');
    fprintf('  -> FCFS done\n');
elseif priority_n == 0
    fprintf('[skip] No FCFS MAT found\n');
else
    fprintf('[skip] FCFS not exported for priority-override runs\n');
end

%% ── Done ─────────────────────────────────────────────────────────────────
fprintf('\n=== Done: %s → %s ===\n', caseName, pfx);
fprintf('Add to HTML AVAILABLE map:\n');
fprintf('  ''%s_optimal'': true,\n', pfx);
if priority_n == 0
    fprintf('  ''%s_fcfs'':    true,\n', pfx);
end
if priority_n == 0
    fprintf('Add to SCENARIO_GROUPS if new group:\n');
    fprintf('  ''%s'': { label:''%s Robots'', scenes:[''%s''] },\n', group, group, scene);
else
    fprintf('Add to SCENARIO_GROUPS scenes list:\n');
    fprintf('  (add ''%s'' to the ''%s'' group scenes array)\n', scene, group);
end
end
