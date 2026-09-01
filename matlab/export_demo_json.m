function export_demo_json(const, x_prev, y_prev, scenario_name, output_file, policy_name, priority_n)
% export_demo_json  Export scheduling result as a JS file for the HTML demo.
%
%   export_demo_json(const, x_prev, y_prev, scenario_name, output_file)
%   export_demo_json(const, x_prev, y_prev, scenario_name, output_file, policy_name)
%   export_demo_json(const, x_prev, y_prev, scenario_name, output_file, policy_name, priority_n)
%
%   output_file : full path including filename, e.g.
%                 fullfile(paths.demoDir, '10r_S1_optimal.js')
%                 (extension is forced to .js regardless of what you pass)
%   policy_name : optional string stored in the JS for reference (default = scenario_name)
%   priority_n  : optional robot index (1-based) that has highest priority (0 = none)
%
% Naming convention expected by the HTML:
%   schedules/{group}_{scene}_{policy}.js
%   e.g.  schedules/10r_S1_optimal.js
%         schedules/10r_S1_p3_optimal.js  (robot 3 priority)

if nargin < 6 || isempty(policy_name), policy_name = scenario_name; end
if nargin < 7 || isempty(priority_n),  priority_n  = 0; end

N = const.N;
result = struct();
result.scenario     = scenario_name;
result.policy       = policy_name;
result.priorityRobot = priority_n;   % 0 = no override; >0 = robot index with forced priority

% ── 顶层全局信息（HTML用于时间轴控制）──
all_alpha0 = cellfun(@(a) a(1), const.alpha_tilde);  % 每辆车最早到达第一路口的时刻
result.t_cut    = min(all_alpha0) - 4.0;   % 动画开始时刻（提前4s显示approach）

% t_final：所有车辆调度后实际离开最后路口的最晚时刻（从y_prev读取）
t_finals = zeros(1, N);
for n = 1:N
    chain        = const.pathInfo_agent_chain{n}{1};
    last_ag      = chain(end-1);          % 最后一个intersection agent（chain末尾为9）
    t_finals(n)  = y_prev{last_ag}{n}(1); % γ：该车离开最后路口的实际时刻
end
result.t_final  = max(t_finals);
result.Dt       = const.Dt;               % road最短行驶时间，供HTML参考
result.vehicles = cell(N,1);

for n = 1:N
    chain      = const.pathInfo_agent_chain{n}{1};
    int_agents = chain(1:2:end-1);   % intersection agents：位置1,3,5,...

    kf = zeros(length(int_agents), 4);
    for k = 1:length(int_agents)
        ag          = int_agents(k);
        kf(k, 1)    = ag;                    % intersection id (1-5)
        kf(k, 2)    = x_prev{ag}{n}(1);     % β: 实际进入时刻
        kf(k, 3)    = y_prev{ag}{n}(1);     % γ: 离开时刻
        kf(k, 4)    = const.pathInfo{n}(1).routeId(k);  % MATLAB routeId (1-12)
    end

    % 用 num2cell 按行拆分，保证 jsonencode 始终输出 [[...],[...]] 嵌套格式
    % （若直接传矩阵，1×3单行矩阵会被编码为 flat 数组 [...]，导致HTML解析错误）
    kf_cell = num2cell(kf, 2);

    % Theoretical deadline: alpha0 + sum(c_k per intersection) + (NI-1)*Dt
    % c_k = sum(IntSpaceDB{ag}.routeDur{routeId_k}) = pure traversal time at intersection k
    deadline_n = const.alpha_tilde{n}(1);
    for k = 1:length(int_agents)
        ag        = int_agents(k);
        rId       = const.pathInfo{n}(1).routeId(k);
        dur_seq   = const.IntSpaceDB{ag}.routeDur{rId};
        deadline_n = deadline_n + sum(dur_seq);
        if k < length(int_agents)
            deadline_n = deadline_n + const.Dt;
        end
    end

    result.vehicles{n} = struct( ...
        'id',       n, ...
        'entrance', const.config{n}.entrance, ...
        'exit',     const.config{n}.exits, ...
        'alpha0',   const.alpha_tilde{n}(1), ...  % 最早到达第一路口
        'deadline', deadline_n, ...               % 理论最早完成时刻（零等待）
        'keyframes', {kf_cell} ...    % cell of row vectors → [[int_id,beta,gamma],...]
    );
end

% ── 输出为 .js 文件（var 赋值），浏览器可通过 <script src> 直接加载，无CORS限制 ──
json_str = jsonencode(result, 'PrettyPrint', true);

% 将输出文件扩展名改为 .js（调用方传入的 output_file 可以是 .json 或 .js）
[fdir, fname, ~] = fileparts(output_file);
js_file = fullfile(fdir, [fname '.js']);

fid = fopen(js_file, 'w');
fprintf(fid, 'var SCHEDULE_DATA = ');
fwrite(fid, json_str, 'char');
fprintf(fid, ';\n');
fclose(fid);

fprintf('Exported: %s  (N=%d vehicles, t_cut=%.2f)\n', js_file, N, result.t_cut);
end
