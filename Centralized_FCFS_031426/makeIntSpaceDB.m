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

    C1 = pi/4;  C2 = 1;  C3 = pi/4;
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
