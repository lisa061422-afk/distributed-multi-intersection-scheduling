function IntSpaceDB = makeIntSpaceDB()
% makeIntSpaceDB  Build space-usage DB for intersections (hard-coded tables).
%
% OUTPUT:
%   IntSpaceDB{int_id} is a struct with fields:
%     .numSpaces, .numRoutes, .routeSpace{r}, .routeDur{r}

    % 你有几个 intersection 就开多大
    numInts = 4;                 % 先按 4 个路口写；不够你再改
    IntSpaceDB = cell(1, numInts);

    % ========== Intersection 1 ==========
    numSpaces = 5;

    Map = [1 1 1 2 2 2 3 3 3 4 4 4;  % route 1..12
           5 2 0 5 3 0 5 4 0 5 1 0;
           3 0 0 4 0 0 1 0 0 2 0 0];

    % 仓库尺寸等比例缩小 traffic (W=20m, v_int=10m/s) → 比例1:12.5
    % W=1.6m, v_int=0.8 m/s → W/v_int = 2.0 (与traffic相同)
    % paper formula: C_arc = pi*W/(8*v_int) = pi/4 ≈ 0.785s  (与traffic完全一致)
    %                C_str = W/(2*v_int)     = 1.0s           (与traffic完全一致)
    W      = 1.6;   % merging zone width (m)
    v_int  = 0.8;   % AMR speed inside merging zone (m/s)
    L_body = 0.6;   % AMR body length (m)  — 0.6×0.6 m square AMR

    C1 = pi * W / (8 * v_int);   % = pi/4 ≈ 0.7854s — left-turn & right-turn sub-tasks
    C2 = W / (2 * v_int);        % = 1.0s            — straight-through sub-tasks
    C3 = C1;
    % 由此得：左转总计 3*C1≈2.356s，直行 2*C2=2.0s，右转 C1≈0.785s
    C = [C1 C2 C3  C1 C2 C3  C1 C2 C3  C1 C2 C3;
         C1 C2 0   C1 C2 0   C1 C2 0   C1 C2 0;
         C1 0  0   C1 0  0   C1 0  0   C1 0  0];

    IntSpaceDB{1} = buildIntSpaceDB(Map, C, numSpaces);

    % ========== Intersection 2 ==========
    % 如果 int2 的 Map/C 和 int1 完全相同，就直接复制：
    IntSpaceDB{2} = IntSpaceDB{1};

    % ========== Intersection 3 ==========
    IntSpaceDB{3} = IntSpaceDB{1};

    % ========== Intersection 4 ==========
    IntSpaceDB{4} = IntSpaceDB{1};

end
