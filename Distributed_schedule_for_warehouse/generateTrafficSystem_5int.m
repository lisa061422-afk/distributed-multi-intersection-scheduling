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
%   N-entry: left(W)=1, straight(S)=2, right(E)=3
%   E-entry: left(N)=4, straight(W)=5, right(S)=6
%   S-entry: left(E)=7, straight(N)=8, right(W)=9
%   W-entry: left(S)=10, straight(E)=11, right(N)=12
%
% subDir = mod(routeId-1, 3) + 1  (1=left, 2=straight, 3=right)

NPORTS = 9;
routeDict = repmat(struct('int', [], 'subDir', [], 'routeId', []), NPORTS, NPORTS);

% Helper: subDir from routeId
sd = @(rid) mod(rid - 1, 3) + 1;

% ===== From port 1 (I1-North) =====
routeDict(1,2) = mk([1],       [1],          sd([1]));           % I1: N→W (left)
routeDict(1,3) = mk([1 2],     [3 12],       sd([3 12]));        % I1:N→E(R) I2:W→N(R)
routeDict(1,4) = mk([1 3],     [2 1],        sd([2 1]));         % I1:N→S(str) I3:N→W(L)
routeDict(1,5) = mk([1 3],     [2 2],        sd([2 2]));         % I1:N→S(str) I3:N→S(str)
routeDict(1,6) = mk([1 2 4],   [3 10 2],     sd([3 10 2]));      % I1:N→E(R) I2:W→S(L) I4:N→S(str)
routeDict(1,7) = mk([1 2 4],   [3 10 3],     sd([3 10 3]));      % I1:N→E(R) I2:W→S(L) I4:N→E(R)
routeDict(1,8) = mk([1 2 5],   [3 11 12],    sd([3 11 12]));     % I1:N→E(R) I2:W→E(str) I5:W→N(R)
routeDict(1,9) = mk([1 2 5],   [3 11 11],    sd([3 11 11]));     % I1:N→E(R) I2:W→E(str) I5:W→E(str)

% ===== From port 2 (I1-West) =====
routeDict(2,1) = mk([1],       [12],         sd([12]));          % I1: W→N (right)
routeDict(2,3) = mk([1 2],     [11 12],      sd([11 12]));       % I1:W→E(str) I2:W→N(R)
routeDict(2,4) = mk([1 3],     [10 1],       sd([10 1]));        % I1:W→S(L) I3:N→W(L)
routeDict(2,5) = mk([1 3],     [10 2],       sd([10 2]));        % I1:W→S(L) I3:N→S(str)
routeDict(2,6) = mk([1 2 4],   [11 10 2],    sd([11 10 2]));     % I1:W→E(str) I2:W→S(L) I4:N→S(str)
routeDict(2,7) = mk([1 2 4],   [11 10 3],    sd([11 10 3]));     % I1:W→E(str) I2:W→S(L) I4:N→E(R)
routeDict(2,8) = mk([1 2 5],   [11 11 12],   sd([11 11 12]));    % I1:W→E(str) I2:W→E(str) I5:W→N(R)
routeDict(2,9) = mk([1 2 5],   [11 11 11],   sd([11 11 11]));    % I1:W→E(str) I2:W→E(str) I5:W→E(str)

% ===== From port 3 (I2-North) =====
routeDict(3,1) = mk([2 1],     [1 4],        sd([1 4]));         % I2:N→W(L) I1:E→N(L)
routeDict(3,2) = mk([2 1],     [1 5],        sd([1 5]));         % I2:N→W(L) I1:E→W(str)
routeDict(3,4) = mk([2 1 3],   [1 6 1],      sd([1 6 1]));       % I2:N→W(L) I1:E→S(R) I3:N→W(L)
routeDict(3,5) = mk([2 1 3],   [1 6 2],      sd([1 6 2]));       % I2:N→W(L) I1:E→S(R) I3:N→S(str)
routeDict(3,6) = mk([2 4],     [2 2],        sd([2 2]));         % I2:N→S(str) I4:N→S(str)
routeDict(3,7) = mk([2 4],     [2 3],        sd([2 3]));         % I2:N→S(str) I4:N→E(R)
routeDict(3,8) = mk([2 5],     [3 12],       sd([3 12]));        % I2:N→E(R) I5:W→N(R)
routeDict(3,9) = mk([2 5],     [3 11],       sd([3 11]));        % I2:N→E(R) I5:W→E(str)

% ===== From port 4 (I3-West) =====
routeDict(4,1) = mk([3 1],     [12 8],       sd([12 8]));        % I3:W→N(R) I1:S→N(str)
routeDict(4,2) = mk([3 1],     [12 9],       sd([12 9]));        % I3:W→N(R) I1:S→W(R)
routeDict(4,3) = mk([3 1 2],   [12 7 12],    sd([12 7 12]));     % I3:W→N(R) I1:S→E(L) I2:W→N(R)
routeDict(4,5) = mk([3],       [10],         sd([10]));          % I3: W→S (left)
routeDict(4,6) = mk([3 4],     [11 10],      sd([11 10]));       % I3:W→E(str) I4:W→S(L)
routeDict(4,7) = mk([3 4],     [11 11],      sd([11 11]));       % I3:W→E(str) I4:W→E(str)
routeDict(4,8) = mk([3 1 2 5], [12 7 11 12], sd([12 7 11 12])); % I3:W→N(R) I1:S→E(L) I2:W→E(str) I5:W→N(R)
routeDict(4,9) = mk([3 1 2 5], [12 7 11 11], sd([12 7 11 11])); % I3:W→N(R) I1:S→E(L) I2:W→E(str) I5:W→E(str)

% ===== From port 5 (I3-South) =====
routeDict(5,1) = mk([3 1],     [8 8],        sd([8 8]));         % I3:S→N(str) I1:S→N(str)
routeDict(5,2) = mk([3 1],     [8 9],        sd([8 9]));         % I3:S→N(str) I1:S→W(R)
routeDict(5,3) = mk([3 1 2],   [8 7 12],     sd([8 7 12]));      % I3:S→N(str) I1:S→E(L) I2:W→N(R)
routeDict(5,4) = mk([3],       [9],          sd([9]));           % I3: S→W (right)
routeDict(5,6) = mk([3 4],     [7 10],       sd([7 10]));        % I3:S→E(L) I4:W→S(L)
routeDict(5,7) = mk([3 4],     [7 11],       sd([7 11]));        % I3:S→E(L) I4:W→E(str)
routeDict(5,8) = mk([3 4 2 5], [7 12 7 12],  sd([7 12 7 12]));  % I3:S→E(L) I4:W→N(R) I2:S→E(L) I5:W→N(R)
routeDict(5,9) = mk([3 4 2 5], [7 12 7 11],  sd([7 12 7 11]));  % I3:S→E(L) I4:W→N(R) I2:S→E(L) I5:W→E(str)

% ===== From port 6 (I4-South) =====
routeDict(6,1) = mk([4 2 1],   [8 9 4],      sd([8 9 4]));       % I4:S→N(str) I2:S→W(R) I1:E→N(L)
routeDict(6,2) = mk([4 2 1],   [8 9 5],      sd([8 9 5]));       % I4:S→N(str) I2:S→W(R) I1:E→W(str)
routeDict(6,3) = mk([4 2],     [8 8],        sd([8 8]));         % I4:S→N(str) I2:S→N(str)
routeDict(6,4) = mk([4 3],     [9 5],        sd([9 5]));         % I4:S→W(R) I3:E→W(str)
routeDict(6,5) = mk([4 3],     [9 6],        sd([9 6]));         % I4:S→W(R) I3:E→S(R)
routeDict(6,7) = mk([4],       [7],          sd([7]));           % I4: S→E (left)
routeDict(6,8) = mk([4 2 5],   [8 7 12],     sd([8 7 12]));      % I4:S→N(str) I2:S→E(L) I5:W→N(R)
routeDict(6,9) = mk([4 2 5],   [8 7 11],     sd([8 7 11]));      % I4:S→N(str) I2:S→E(L) I5:W→E(str)

% ===== From port 7 (I4-East) =====
routeDict(7,1) = mk([4 2 1],   [4 9 4],      sd([4 9 4]));       % I4:E→N(L) I2:S→W(R) I1:E→N(L)
routeDict(7,2) = mk([4 2 1],   [4 9 5],      sd([4 9 5]));       % I4:E→N(L) I2:S→W(R) I1:E→W(str)
routeDict(7,3) = mk([4 2],     [4 8],        sd([4 8]));         % I4:E→N(L) I2:S→N(str)
routeDict(7,4) = mk([4 3],     [5 5],        sd([5 5]));         % I4:E→W(str) I3:E→W(str)
routeDict(7,5) = mk([4 3],     [5 6],        sd([5 6]));         % I4:E→W(str) I3:E→S(R)
routeDict(7,6) = mk([4],       [6],          sd([6]));           % I4: E→S (right)
routeDict(7,8) = mk([4 2 5],   [4 7 12],     sd([4 7 12]));      % I4:E→N(L) I2:S→E(L) I5:W→N(R)
routeDict(7,9) = mk([4 2 5],   [4 7 11],     sd([4 7 11]));      % I4:E→N(L) I2:S→E(L) I5:W→E(str)

% ===== From port 8 (I5-North) =====
routeDict(8,9) = mk([5],       [3],          sd([3]));           % I5: N→E (right)
routeDict(8,1) = mk([5 2 1],   [1 5 4],      sd([1 5 4]));       % I5:N→W(L) I2:E→W(str) I1:E→N(L)
routeDict(8,2) = mk([5 2 1],   [1 5 5],      sd([1 5 5]));       % I5:N→W(L) I2:E→W(str) I1:E→W(str)
routeDict(8,3) = mk([5 2],     [1 4],        sd([1 4]));         % I5:N→W(L) I2:E→N(L)
routeDict(8,4) = mk([5 2 1 3], [1 5 6 1],    sd([1 5 6 1]));     % I5:N→W(L) I2:E→W(str) I1:E→S(R) I3:N→W(L)
routeDict(8,5) = mk([5 2 1 3], [1 5 6 2],    sd([1 5 6 2]));     % I5:N→W(L) I2:E→W(str) I1:E→S(R) I3:N→S(str)
routeDict(8,6) = mk([5 2 4],   [1 6 2],      sd([1 6 2]));       % I5:N→W(L) I2:E→S(R) I4:N→S(str)
routeDict(8,7) = mk([5 2 4],   [1 6 3],      sd([1 6 3]));       % I5:N→W(L) I2:E→S(R) I4:N→E(R)

% ===== From port 9 (I5-East) =====
routeDict(9,8) = mk([5],       [4],          sd([4]));           % I5: E→N (left)
routeDict(9,1) = mk([5 2 1],   [5 5 4],      sd([5 5 4]));       % I5:E→W(str) I2:E→W(str) I1:E→N(L)
routeDict(9,2) = mk([5 2 1],   [5 5 5],      sd([5 5 5]));       % I5:E→W(str) I2:E→W(str) I1:E→W(str)
routeDict(9,3) = mk([5 2],     [5 4],        sd([5 4]));         % I5:E→W(str) I2:E→N(L)
routeDict(9,4) = mk([5 2 1 3], [5 5 6 1],    sd([5 5 6 1]));     % I5:E→W(str) I2:E→W(str) I1:E→S(R) I3:N→W(L)
routeDict(9,5) = mk([5 2 1 3], [5 5 6 2],    sd([5 5 6 2]));     % I5:E→W(str) I2:E→W(str) I1:E→S(R) I3:N→S(str)
routeDict(9,6) = mk([5 2 4],   [5 6 2],      sd([5 6 2]));       % I5:E→W(str) I2:E→S(R) I4:N→S(str)
routeDict(9,7) = mk([5 2 4],   [5 6 3],      sd([5 6 3]));       % I5:E→W(str) I2:E→S(R) I4:N→E(R)

end

% ── helper ────────────────────────────────────────────────────────────────
function s = mk(int, routeId, subDir)
    s = struct('int', int, 'subDir', subDir, 'routeId', routeId);
end
