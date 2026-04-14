function routeDict = generateTrafficSystem()
% 手工字典：routeDB(e, x) 给出从 entrance=e 到 exit=x 的完整手工配置

routeDict = repmat(struct( ...
    'int', [], ...          % intersections visited
    'subDir', [], ...       % optional
    'routeId', []), 8, 8);

    % 手动填充 (entrance, exit) → (int, subDir,routeId,spaces,c)
    % ===== 入口 1 =====
    routeDict(1,2) = struct('int', [1 2], 'subDir', [3 3],'routeId',[3 12]);  % exit 2
    routeDict(1,3) = struct('int', [1 2], 'subDir', [3 2],'routeId',[3 11]);  % exit 3
    routeDict(1,4) = struct('int', [1 2 3], 'subDir', [3 1 3],'routeId',[3 10 3]);  % exit 4
    routeDict(1,5) = struct('int', [1 2 3], 'subDir', [3 1 2],'routeId',[3 10 2]);  % exit 5
    routeDict(1,6) = struct('int', [1 4], 'subDir', [2 2],'routeId',[2 2]);  % exit 6
    routeDict(1,7) = struct('int', [1 4], 'subDir', [2 1],'routeId',[2 1]);  % exit 7
    routeDict(1,8) = struct('int', [1], 'subDir', [1],'routeId',[1]);  % exit 8

    % ===== 入口 2 =====
    routeDict(2,1) = struct('int', [2 1], 'subDir', [1 1],'routeId',[1 4]);  
    routeDict(2,3) = struct('int', [2], 'subDir', [3],'routeId',[3]);  
    routeDict(2,4) = struct('int', [2 3], 'subDir', [2 3],'routeId',[2 3]);  
    routeDict(2,5) = struct('int', [2 3], 'subDir', [2 2],'routeId',[2 2]);  
    routeDict(2,6) = struct('int', [2 3 4], 'subDir', [2 1 3],'routeId',[2 1 6]);  
    routeDict(2,7) = struct('int', [2 3 4], 'subDir', [2 1 2],'routeId',[2 1 5]);  
    routeDict(2,8) = struct('int', [2 1], 'subDir', [1 2],'routeId',[1 5]);  

    % ===== 入口 3 =====
    routeDict(3,1) = struct('int', [2 1], 'subDir', [2 1],'routeId',[5 4]);  
    routeDict(3,2) = struct('int', [2], 'subDir', [1],'routeId',[4]);  
    routeDict(3,4) = struct('int', [2 3], 'subDir', [3 3],'routeId',[6 3]);  
    routeDict(3,5) = struct('int', [2 3], 'subDir', [3 2],'routeId',[6 2]);  
    routeDict(3,6) = struct('int', [2 3 4], 'subDir', [3 1 3],'routeId',[6 1 6]);  
    routeDict(3,7) = struct('int', [2 3 4], 'subDir', [3 1 2],'routeId',[6 1 5]); 
    routeDict(3,8) = struct('int', [2 1], 'subDir', [2 2],'routeId',[5 5]); 

    % ===== 入口 4 =====
    routeDict(4,1) = struct('int', [3 4 1], 'subDir', [2 1 2],'routeId',[5 4 8]);  
    routeDict(4,2) = struct('int', [3 2], 'subDir', [1 2],'routeId',[4 8]);  
    routeDict(4,3) = struct('int', [3 2], 'subDir', [1 1],'routeId',[4 7]);  
    routeDict(4,5) = struct('int', [3], 'subDir', [3],'routeId',[6]);  
    routeDict(4,6) = struct('int', [3 4], 'subDir', [2 3],'routeId',[5 6]);  
    routeDict(4,7) = struct('int', [3 4], 'subDir', [2 2],'routeId',[5 5]); 
    routeDict(4,8) = struct('int', [3 4 1], 'subDir', [2 1 3],'routeId',[5 4 9]); 

    % ===== 入口 5 =====
    routeDict(5,1) = struct('int', [3 4 1], 'subDir', [3 1 2],'routeId',[9 4 8]);  
    routeDict(5,2) = struct('int', [3 2], 'subDir', [2 2],'routeId',[8 8]);  
    routeDict(5,3) = struct('int', [3 2], 'subDir', [2 1],'routeId',[8 7]);  
    routeDict(5,4) = struct('int', [3], 'subDir', [1],'routeId',[7]);  
    routeDict(5,6) = struct('int', [3 4], 'subDir', [3 3],'routeId',[9 6]);  
    routeDict(5,7) = struct('int', [3 4], 'subDir', [3 2],'routeId',[9 5]);  
    routeDict(5,8) = struct('int', [3 4 1], 'subDir', [3 1 3],'routeId',[9 4 9]); 

    % ===== 入口 6 =====
    routeDict(6,1) = struct('int', [4 1], 'subDir', [2 2],'routeId',[8 8]);  
    routeDict(6,2) = struct('int', [4 1 2], 'subDir', [2 1 3],'routeId',[8 7 12]);  
    routeDict(6,3) = struct('int', [4 1 2], 'subDir', [2 1 2],'routeId',[8 7 11]);  
    routeDict(6,4) = struct('int', [4 3], 'subDir', [1 2],'routeId',[7 11]);  
    routeDict(6,5) = struct('int', [4 3], 'subDir', [1 1],'routeId',[7 10]);  
    routeDict(6,7) = struct('int', [4], 'subDir', [3],'routeId',[9]);  
    routeDict(6,8) = struct('int', [4 1], 'subDir', [2 3],'routeId',[8 9]); 

    % ===== 入口 7 =====
    routeDict(7,1) = struct('int', [4 1], 'subDir', [3 2],'routeId',[12 8]);  
    routeDict(7,2) = struct('int', [4 1 2], 'subDir', [3 1 3],'routeId',[12 7 12]);  
    routeDict(7,3) = struct('int', [4 1 2], 'subDir', [3 1 2],'routeId',[12 7 11]);  
    routeDict(7,4) = struct('int', [4 3], 'subDir', [2 2],'routeId',[11 11]);  
    routeDict(7,5) = struct('int', [4 3], 'subDir', [2 1],'routeId',[11 10]);  
    routeDict(7,6) = struct('int', [4], 'subDir', [1],'routeId',[10]); 
    routeDict(7,8) = struct('int', [4 1], 'subDir', [3 3],'routeId',[12 9]); 

    % ===== 入口 8 =====
    routeDict(8,1) = struct('int', [1], 'subDir', [3],'routeId',[12]);  
    routeDict(8,2) = struct('int', [1 2], 'subDir', [2 3],'routeId',[11 12]);  
    routeDict(8,3) = struct('int', [1 2], 'subDir', [2 2],'routeId',[11 11]);  
    routeDict(8,4) = struct('int', [1 2 3], 'subDir', [2 1 3],'routeId',[11 10 3]);  
    routeDict(8,5) = struct('int', [1 2 3], 'subDir', [2 1 2],'routeId',[11 10 2]);  
    routeDict(8,6) = struct('int', [1 4], 'subDir', [1 2],'routeId',[10 2]); 
    routeDict(8,7) = struct('int', [1 4], 'subDir', [1 1],'routeId',[10 1]);  
end
