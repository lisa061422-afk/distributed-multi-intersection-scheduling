function intCfg = makeIntersectionConfig()
% makeIntersectionConfig  Hard-coded Map/C for a standard 4-arm intersection.
%
%   12 routes: 4 entrance arms × 3 exit directions (left / straight / right).
%   Route indexing (matches makeIntSpaceDB in distributed repo):
%     Arm 1 entrances: routes  1(left), 2(straight), 3(right)
%     Arm 2 entrances: routes  4(left), 5(straight), 6(right)
%     Arm 3 entrances: routes  7(left), 8(straight), 9(right)
%     Arm 4 entrances: routes 10(left),11(straight),12(right)
%
%   Map(s, r) = space index used at sub-task s of route r  (0 = no sub-task)
%   C(s, r)   = duration of that sub-task (s)
%
% OUTPUT
%   intCfg.Map       — 3×12 matrix
%   intCfg.C         — 3×12 matrix
%   intCfg.N         — 12  (number of routes)
%   intCfg.M         — 5   (number of spaces)
%   intCfg.S         — 3   (max sub-tasks per route)

    intCfg.N = 12;
    intCfg.M = 5;
    intCfg.S = 3;

    % ── Space occupancy ───────────────────────────────────────────────────
    % Row = sub-task index (1..3); Column = route index (1..12)
    % 0 means the route has no sub-task at that row.
    intCfg.Map = ...
        [1  1  1  2  2  2  3  3  3  4  4  4;   % sub-task 1
         5  2  0  5  3  0  5  4  0  5  1  0;   % sub-task 2
         3  0  0  4  0  0  1  0  0  2  0  0];  % sub-task 3

    % ── Sub-task durations ────────────────────────────────────────────────
    % Using traffic-scale constants (consistent with original 3-route code):
    %   arc sub-task  C_arc = pi/4  ≈ 0.785 s   (left-turn arcs, right-turn)
    %   straight      C_str = 1.0 s             (straight-through)
    Ca = pi/4;   % arc (left-turn phases 1&3, right-turn phase 1)
    Cs = 1.0;    % straight

    % Route structure per arm:
    %   left   (col 1,4,7,10): [Ca Cs Ca; Ca Cs 0; Ca 0 0] transposed → 3 sub-tasks
    %   straight(col 2,5,8,11): [Ca Cs; Ca Cs; 0 0] → 2 sub-tasks
    %   right  (col 3,6,9,12): [Ca; 0; 0] → 1 sub-task
    block = [Ca  Cs  Ca   Ca  Cs  Ca   Ca  Cs  Ca   Ca  Cs  Ca;
             Ca  Cs  0    Ca  Cs  0    Ca  Cs  0    Ca  Cs  0;
             Ca  0   0    Ca  0   0    Ca  0   0    Ca  0   0];
    intCfg.C = block;

end
