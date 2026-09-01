function routeDict = generateTrafficSystem_5int()
% Route dictionary for 5-intersection traffic map.
%
% Topology:
%          port1(N)
%            |
% port2(W)--[I1]--road6--[I2]--road7--[I5]--port9(E)
%            |    port3(N) |          port8(N)
%           road8         road9
%            |              |
% port4(W)--[I3]--road10--[I4]--port7(E)
%            |               |
%           port5(S)        port6(S)
%
% Intersections: I1=1, I2=2, I3=3, I4=4, I5=5
% Road agents:   I1-I2=6, I2-I5=7, I1-I3=8, I2-I4=9, I3-I4=10
% Terminal:      11
%
% External ports (9 total):
%   1 = I1-North   2 = I1-West    3 = I2-North
%   4 = I3-West    5 = I3-South   6 = I4-South
%   7 = I4-East    8 = I5-North   9 = I5-East
%
% routeId encoding: entry_direction × turn_type
%   N-entry: left(E)=7, straight(S)=8, right(W)=9
%   E-entry: left(S)=4, straight(W)=5, right(N)=6
%   S-entry: left(W)=1, straight(N)=2, right(E)=3
%   W-entry: left(N)=10, straight(E)=11, right(S)=12
%
% subDir = mod(routeId-1, 3) + 1  (1=left, 2=straight, 3=right)

NPORTS = 9;
routeDict = repmat(struct('int', [], 'subDir', [], 'routeId', []), NPORTS, NPORTS);

% Helper: subDir from routeId
sd = @(rid) mod(rid - 1, 3) + 1;

% ===== From port 1 (I1-North) =====
routeDict(1,2) = mk([1],       [9],          sd([9]));           % I1: N→W(R)
routeDict(1,3) = mk([1 2],     [7 10],       sd([7 10]));        % I1:N→E(L) I2:W→N(L)
routeDict(1,4) = mk([1 3],     [8 9],        sd([8 9]));         % I1:N→S(S) I3:N→W(R)
routeDict(1,5) = mk([1 3],     [8 8],        sd([8 8]));         % I1:N→S(S) I3:N→S(S)
routeDict(1,6) = mk([1 2 4],   [7 12 8],     sd([7 12 8]));      % I1:N→E(L) I2:W→S(R) I4:N→S(S)
routeDict(1,7) = mk([1 2 4],   [7 12 7],     sd([7 12 7]));      % I1:N→E(L) I2:W→S(R) I4:N→E(L)
routeDict(1,8) = mk([1 2 5],   [7 11 10],    sd([7 11 10]));     % I1:N→E(L) I2:W→E(S) I5:W→N(L)
routeDict(1,9) = mk([1 2 5],   [7 11 11],    sd([7 11 11]));     % I1:N→E(L) I2:W→E(S) I5:W→E(S)

% ===== From port 2 (I1-West) =====
routeDict(2,1) = mk([1],       [10],         sd([10]));          % I1: W→N(L)
routeDict(2,3) = mk([1 2],     [11 10],      sd([11 10]));       % I1:W→E(S) I2:W→N(L)
routeDict(2,4) = mk([1 3],     [12 9],       sd([12 9]));        % I1:W→S(R) I3:N→W(R)
routeDict(2,5) = mk([1 3],     [12 8],       sd([12 8]));        % I1:W→S(R) I3:N→S(S)
routeDict(2,6) = mk([1 2 4],   [11 12 8],    sd([11 12 8]));     % I1:W→E(S) I2:W→S(R) I4:N→S(S)
routeDict(2,7) = mk([1 2 4],   [11 12 7],    sd([11 12 7]));     % I1:W→E(S) I2:W→S(R) I4:N→E(L)
routeDict(2,8) = mk([1 2 5],   [11 11 10],   sd([11 11 10]));    % I1:W→E(S) I2:W→E(S) I5:W→N(L)
routeDict(2,9) = mk([1 2 5],   [11 11 11],   sd([11 11 11]));    % I1:W→E(S) I2:W→E(S) I5:W→E(S)

% ===== From port 3 (I2-North) =====
routeDict(3,1) = mk([2 1],     [9 6],        sd([9 6]));         % I2:N→W(R) I1:E→N(R)
routeDict(3,2) = mk([2 1],     [9 5],        sd([9 5]));         % I2:N→W(R) I1:E→W(S)
routeDict(3,4) = mk([2 1 3],   [9 4 9],      sd([9 4 9]));       % I2:N→W(R) I1:E→S(L) I3:N→W(R)
routeDict(3,5) = mk([2 1 3],   [9 4 8],      sd([9 4 8]));       % I2:N→W(R) I1:E→S(L) I3:N→S(S)
routeDict(3,6) = mk([2 4],     [8 8],        sd([8 8]));         % I2:N→S(S) I4:N→S(S)
routeDict(3,7) = mk([2 4],     [8 7],        sd([8 7]));         % I2:N→S(S) I4:N→E(L)
routeDict(3,8) = mk([2 5],     [7 10],       sd([7 10]));        % I2:N→E(L) I5:W→N(L)
routeDict(3,9) = mk([2 5],     [7 11],       sd([7 11]));        % I2:N→E(L) I5:W→E(S)

% ===== From port 4 (I3-West) =====
routeDict(4,1) = mk([3 1],     [10 2],       sd([10 2]));        % I3:W→N(L) I1:S→N(S)
routeDict(4,2) = mk([3 1],     [10 1],       sd([10 1]));        % I3:W→N(L) I1:S→W(L)
routeDict(4,3) = mk([3 1 2],   [10 3 10],    sd([10 3 10]));     % I3:W→N(L) I1:S→E(R) I2:W→N(L)
routeDict(4,5) = mk([3],       [12],         sd([12]));          % I3: W→S(R)
routeDict(4,6) = mk([3 4],     [11 12],      sd([11 12]));       % I3:W→E(S) I4:W→S(R)
routeDict(4,7) = mk([3 4],     [11 11],      sd([11 11]));       % I3:W→E(S) I4:W→E(S)
routeDict(4,8) = mk([3 1 2 5], [10 3 11 10], sd([10 3 11 10])); % I3:W→N(L) I1:S→E(R) I2:W→E(S) I5:W→N(L)
routeDict(4,9) = mk([3 1 2 5], [10 3 11 11], sd([10 3 11 11])); % I3:W→N(L) I1:S→E(R) I2:W→E(S) I5:W→E(S)

% ===== From port 5 (I3-South) =====
routeDict(5,1) = mk([3 1],     [2 2],        sd([2 2]));         % I3:S→N(S) I1:S→N(S)
routeDict(5,2) = mk([3 1],     [2 1],        sd([2 1]));         % I3:S→N(S) I1:S→W(L)
routeDict(5,3) = mk([3 1 2],   [2 3 10],     sd([2 3 10]));      % I3:S→N(S) I1:S→E(R) I2:W→N(L)
routeDict(5,4) = mk([3],       [1],          sd([1]));           % I3: S→W(L)
routeDict(5,6) = mk([3 4],     [3 12],       sd([3 12]));        % I3:S→E(R) I4:W→S(R)
routeDict(5,7) = mk([3 4],     [3 11],       sd([3 11]));        % I3:S→E(R) I4:W→E(S)
routeDict(5,8) = mk([3 4 2 5], [3 10 3 10],  sd([3 10 3 10]));  % I3:S→E(R) I4:W→N(L) I2:S→E(R) I5:W→N(L)
routeDict(5,9) = mk([3 4 2 5], [3 10 3 11],  sd([3 10 3 11]));  % I3:S→E(R) I4:W→N(L) I2:S→E(R) I5:W→E(S)

% ===== From port 6 (I4-South) =====
routeDict(6,1) = mk([4 2 1],   [2 1 6],      sd([2 1 6]));       % I4:S→N(S) I2:S→W(L) I1:E→N(R)
routeDict(6,2) = mk([4 2 1],   [2 1 5],      sd([2 1 5]));       % I4:S→N(S) I2:S→W(L) I1:E→W(S)
routeDict(6,3) = mk([4 2],     [2 2],        sd([2 2]));         % I4:S→N(S) I2:S→N(S)
routeDict(6,4) = mk([4 3],     [1 5],        sd([1 5]));         % I4:S→W(L) I3:E→W(S)
routeDict(6,5) = mk([4 3],     [1 4],        sd([1 4]));         % I4:S→W(L) I3:E→S(L)
routeDict(6,7) = mk([4],       [3],          sd([3]));           % I4: S→E(R)
routeDict(6,8) = mk([4 2 5],   [2 3 10],     sd([2 3 10]));      % I4:S→N(S) I2:S→E(R) I5:W→N(L)
routeDict(6,9) = mk([4 2 5],   [2 3 11],     sd([2 3 11]));      % I4:S→N(S) I2:S→E(R) I5:W→E(S)

% ===== From port 7 (I4-East) =====
routeDict(7,1) = mk([4 2 1],   [6 1 6],      sd([6 1 6]));       % I4:E→N(R) I2:S→W(L) I1:E→N(R)
routeDict(7,2) = mk([4 2 1],   [6 1 5],      sd([6 1 5]));       % I4:E→N(R) I2:S→W(L) I1:E→W(S)
routeDict(7,3) = mk([4 2],     [6 2],        sd([6 2]));         % I4:E→N(R) I2:S→N(S)
routeDict(7,4) = mk([4 3],     [5 5],        sd([5 5]));         % I4:E→W(S) I3:E→W(S)
routeDict(7,5) = mk([4 3],     [5 4],        sd([5 4]));         % I4:E→W(S) I3:E→S(L)
routeDict(7,6) = mk([4],       [4],          sd([4]));           % I4: E→S(L)
routeDict(7,8) = mk([4 2 5],   [6 3 10],     sd([6 3 10]));      % I4:E→N(R) I2:S→E(R) I5:W→N(L)
routeDict(7,9) = mk([4 2 5],   [6 3 11],     sd([6 3 11]));      % I4:E→N(R) I2:S→E(R) I5:W→E(S)

% ===== From port 8 (I5-North) =====
routeDict(8,9) = mk([5],       [7],          sd([7]));           % I5: N→E(L)
routeDict(8,1) = mk([5 2 1],   [9 5 6],      sd([9 5 6]));       % I5:N→W(R) I2:E→W(S) I1:E→N(R)
routeDict(8,2) = mk([5 2 1],   [9 5 5],      sd([9 5 5]));       % I5:N→W(R) I2:E→W(S) I1:E→W(S)
routeDict(8,3) = mk([5 2],     [9 6],        sd([9 6]));         % I5:N→W(R) I2:E→N(R)
routeDict(8,4) = mk([5 2 1 3], [9 5 4 9],    sd([9 5 4 9]));     % I5:N→W(R) I2:E→W(S) I1:E→S(L) I3:N→W(R)
routeDict(8,5) = mk([5 2 1 3], [9 5 4 8],    sd([9 5 4 8]));     % I5:N→W(R) I2:E→W(S) I1:E→S(L) I3:N→S(S)
routeDict(8,6) = mk([5 2 4],   [9 4 8],      sd([9 4 8]));       % I5:N→W(R) I2:E→S(L) I4:N→S(S)
routeDict(8,7) = mk([5 2 4],   [9 4 7],      sd([9 4 7]));       % I5:N→W(R) I2:E→S(L) I4:N→E(L)

% ===== From port 9 (I5-East) =====
routeDict(9,8) = mk([5],       [6],          sd([6]));           % I5: E→N(R)
routeDict(9,1) = mk([5 2 1],   [5 5 6],      sd([5 5 6]));       % I5:E→W(S) I2:E→W(S) I1:E→N(R)
routeDict(9,2) = mk([5 2 1],   [5 5 5],      sd([5 5 5]));       % I5:E→W(S) I2:E→W(S) I1:E→W(S)
routeDict(9,3) = mk([5 2],     [5 6],        sd([5 6]));         % I5:E→W(S) I2:E→N(R)
routeDict(9,4) = mk([5 2 1 3], [5 5 4 9],    sd([5 5 4 9]));     % I5:E→W(S) I2:E→W(S) I1:E→S(L) I3:N→W(R)
routeDict(9,5) = mk([5 2 1 3], [5 5 4 8],    sd([5 5 4 8]));     % I5:E→W(S) I2:E→W(S) I1:E→S(L) I3:N→S(S)
routeDict(9,6) = mk([5 2 4],   [5 4 8],      sd([5 4 8]));       % I5:E→W(S) I2:E→S(L) I4:N→S(S)
routeDict(9,7) = mk([5 2 4],   [5 4 7],      sd([5 4 7]));       % I5:E→W(S) I2:E→S(L) I4:N→E(L)

end

% ── helper ────────────────────────────────────────────────────────────────
function s = mk(int, routeId, subDir)
    s = struct('int', int, 'subDir', subDir, 'routeId', routeId);
end
