function pathInfo = getVehiclePaths_5int(config)
% getVehiclePaths_5int  Path lookup for the 5-intersection traffic map.
% Drop-in replacement for getVehiclePaths when using generateTrafficSystem_5int.

routeDict = generateTrafficSystem_5int();

N = numel(config);
pathInfo = cell(1, N);

for n = 1:N
    e    = config{n}.entrance;
    exits = config{n}.exits;
    Kn   = numel(exits);
    pathInfo{n} = repmat(struct('int', [], 'subDir', [], 'routeId', []), 1, Kn);

    for k = 1:Kn
        x = exits(k);
        if e == x
            error('getVehiclePaths_5int: entrance %d == exit %d for vehicle %d', e, x, n);
        end
        r = routeDict(e, x);
        if isempty(r.int)
            error('getVehiclePaths_5int: no route for (%d→%d). Fill generateTrafficSystem_5int.', e, x);
        end
        pathInfo{n}(k).int     = r.int;
        pathInfo{n}(k).subDir  = r.subDir;
        pathInfo{n}(k).routeId = r.routeId;
    end
end
end
