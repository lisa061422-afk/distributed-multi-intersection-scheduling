function export_demo_json_fcfs(const_fcfs, DATA, scenario_name, output_file, policy_name)
% export_demo_json_fcfs  Export centralized FCFS result as .js for HTML demo.
%
%   export_demo_json_fcfs(const_fcfs, DATA, scenario_name, output_file)
%   export_demo_json_fcfs(const_fcfs, DATA, scenario_name, output_file, policy_name)
%
%   const_fcfs : the 'const' struct from MAIN_Centralized_FCFS
%   DATA       : output of build_centralized_schedule_data(NODES, Path_min, const)
%   output_file: full path, e.g. '...\schedules\10r_S1_fcfs.js'
%
% Naming convention: schedules/{group}_{scene}_{policy}.js

if nargin < 5, policy_name = scenario_name; end

N   = const_fcfs.N;
Dt  = const_fcfs.Dt;
Veh = const_fcfs.Veh;
alpha_tilde = const_fcfs.alpha_tilde;  % cell(N,1), each element is scalar

result = struct();
result.scenario = scenario_name;
result.policy   = policy_name;

% ── Global timing ──────────────────────────────────────────────────────────
all_alpha0 = cellfun(@(a) a(1), alpha_tilde);
result.t_cut   = min(all_alpha0) - 4.0;

t_finals = zeros(1, N);
for n = 1:N
    t_finals(n) = DATA.gamma_all{n}(Veh(n).NI);   % last intersection exit time
end
result.t_final = max(t_finals);
result.Dt      = Dt;
result.vehicles = cell(N, 1);

% ── Per-vehicle keyframes ───────────────────────────────────────────────────
for n = 1:N
    intSeq = Veh(n).intSeq;   % sequence of intersection IDs visited
    NI_n   = numel(intSeq);

    % Build keyframes and deadline in one pass
    kf = zeros(NI_n, 3);
    deadline_n = alpha_tilde{n}(1);
    for k = 1:NI_n
        p       = intSeq(k);
        gamma_k = DATA.gamma_all{n}(k);
        rId     = Veh(n).routeId(k);
        dur_seq = const_fcfs.IntSpaceDB{p}.routeDur{rId};
        c_k     = sum(dur_seq);

        % True service entry time (beta) from NewNode_global: start_time = gamma - c_total
        beta_k  = gamma_k - c_k;

        kf(k, 1) = p;        % intersection id (1-4)
        kf(k, 2) = beta_k;   % β: true service start (not earliest arrival alpha_k)
        kf(k, 3) = gamma_k;  % γ: exit time

        % Accumulate theoretical deadline (zero-wait reference)
        deadline_n = deadline_n + c_k;
        if k < NI_n
            deadline_n = deadline_n + Dt;
        end
    end

    % num2cell row-by-row to ensure jsonencode writes [[...],[...]] not [...]
    kf_cell = num2cell(kf, 2);

    result.vehicles{n} = struct( ...
        'id',       n, ...
        'entrance', Veh(n).entrance, ...
        'exit',     Veh(n).exit, ...
        'alpha0',   alpha_tilde{n}(1), ...
        'deadline', deadline_n, ...    % 理论最早完成时刻（零等待）
        'keyframes', {kf_cell} ...
    );
end

% ── Write .js file ──────────────────────────────────────────────────────────
json_str = jsonencode(result, 'PrettyPrint', true);

[fdir, fname, ~] = fileparts(output_file);
js_file = fullfile(fdir, [fname '.js']);

fid = fopen(js_file, 'w');
fprintf(fid, 'var SCHEDULE_DATA = ');
fwrite(fid, json_str, 'char');
fprintf(fid, ';\n');
fclose(fid);

fprintf('Exported: %s  (N=%d vehicles, t_cut=%.2f)\n', js_file, N, result.t_cut);
end
