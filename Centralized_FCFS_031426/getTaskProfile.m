function task = getTaskProfile(n, taskIdx, const)

Veh = const.Veh;
IntSpaceDB = const.IntSpaceDB;
spacePerInt = const.spacePerInt;
Smax = const.Smax;

task = struct();
task.C = zeros(Smax,1);
task.Map = zeros(Smax,1);

% 第 taskIdx 个路口任务
int_id   = Veh(n).intSeq(taskIdx);
route_id = Veh(n).routeId(taskIdx);

% 当前路口内的局部 space 和 duration
local_spaces = IntSpaceDB{int_id}.routeSpace{route_id};
local_durs   = IntSpaceDB{int_id}.routeDur{route_id};

% 映射成全局 space 编号
global_spaces = local_spaces + (int_id - 1) * spacePerInt;

L = numel(local_durs);
task.C(1:L)   = local_durs(:);
task.Map(1:L) = global_spaces(:);

task.int_id   = int_id;
task.route_id = route_id;
task.L        = L;
end
% task = getTaskProfile(n, ni(n)+1, const);