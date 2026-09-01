function pathInfo = getVehiclePaths(config)
% config{n}.entrance, config{n}.exits (vector)
% pathInfo{n}(k) gives one vehicle (the k-th in exits list)

routeDict = generateTrafficSystem();

N = numel(config);
pathInfo = cell(1, N);

for n = 1:N
    e = config{n}.entrance;
    exits = config{n}.exits;
    Kn = numel(exits);
    pathInfo{n} = repmat(struct('int', [], 'subDir', [], 'routeId', []), 1, Kn);

    for k = 1:Kn
        x = exits(k);
        r = routeDict(e, x);

        if isempty(r.int)
            error('No routeDict(%d,%d) entry. Please hand-fill it in generateTrafficSystem().', e, x);
        end

        pathInfo{n}(k).int = r.int;
        pathInfo{n}(k).subDir = r.subDir;
        pathInfo{n}(k).routeId = r.routeId;  % 新增，后续local可用
    end
end
end
