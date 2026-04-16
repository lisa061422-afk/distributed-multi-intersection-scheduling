function [config, vehicleList, stats] = generateBalancedTrafficConfig_5int(numVehicles, varargin)
%GENERATEBALANCEDTRAFFICCONFIG_5INT
% 5-intersection version of generateBalancedTrafficConfig.
% Works with the 9-port, 5-intersection topology:
%   I1=1, I2=2, I3=3, I4=4, I5=5
%   Ports 1-9 (I1-N, I1-W, I2-N, I3-W, I3-S, I4-S, I4-E, I5-N, I5-E)
%
% Output config format (same as manual input):
%   config = { struct('entrance', 1, 'exits', [7 3]), ... }

    p = inputParser;
    addRequired(p, 'numVehicles', @(x)isnumeric(x) && isscalar(x) && x>=1);

    addParameter(p, 'MaxPerEntrance',     ceil(numVehicles/9)+1, @(x)isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'MaxPerIntersection', Inf,  @(x)isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'MaxPerStage',        Inf,  @(x)isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'EntrancePenalty',    0.30, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'StagePenalty',       1.00, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'TotalIntPenalty',    0.35, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'SameODPenalty',      0.15, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'LongRoutePenalty',   0.25, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'LongRouteProb',      0.08, @(x)isnumeric(x) && isscalar(x) && x>=0 && x<=1);
    addParameter(p, 'RandomTieBreak',     true, @(x)islogical(x) && isscalar(x));
    addParameter(p, 'Seed',               [],   @(x)isempty(x) || (isnumeric(x) && isscalar(x)));
    addParameter(p, 'MaxSingleInt',       1,    @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'MinLongInt',         1,    @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'MaxLongInt',         3,    @(x)isnumeric(x) && isscalar(x) && x>=0);
    parse(p, numVehicles, varargin{:});

    maxPerEntrance = p.Results.MaxPerEntrance;
    maxPerInt      = p.Results.MaxPerIntersection;
    maxPerStage    = p.Results.MaxPerStage;
    wEntrance      = p.Results.EntrancePenalty;
    wStage         = p.Results.StagePenalty;
    wTotal         = p.Results.TotalIntPenalty;
    wSameOD        = p.Results.SameODPenalty;
    wLong          = p.Results.LongRoutePenalty;
    longRouteProb  = p.Results.LongRouteProb;
    randomTieBreak = p.Results.RandomTieBreak;
    seed           = p.Results.Seed;
    maxSingleInt   = p.Results.MaxSingleInt;
    minLongInt     = p.Results.MinLongInt;
    maxLongInt     = p.Results.MaxLongInt;

    if ~isempty(seed), rng(seed); end

    NUM_ENT = 9;   % 9 external ports
    NUM_INT = 5;   % 5 intersections
    MAX_STG = 4;   % max intersections per route in this topology

    routeDict  = generateTrafficSystem_5int();
    candidates = buildCandidateList_5int(routeDict);

    if isempty(candidates)
        error('No valid routes found in routeDict.');
    end

    routeLens   = arrayfun(@(s) numel(s.ints), candidates);
    singleCands = candidates(routeLens == 1);
    medCands    = candidates(routeLens == 2);
    longCands   = candidates(routeLens >= 3);

    % State
    entranceLoad = zeros(1, NUM_ENT);
    totalIntLoad = zeros(1, NUM_INT);
    stageLoad    = zeros(NUM_INT, MAX_STG);
    singleCount  = 0;
    longCount    = 0;

    vehicleList = repmat(struct( ...
        'entrance', [], 'exit', [], ...
        'ints', [], 'routeId', [], 'subDir', []), 1, numVehicles);

    for k = 1:numVehicles
        remaining = numVehicles - k + 1;

        mustFillLong = (longCount < minLongInt) && ...
                       (remaining <= (minLongInt - longCount)) && ...
                       ~isempty(longCands);

        if mustFillLong
            pool = longCands;
        elseif longCount >= maxLongInt
            pool = [singleCands, medCands];
        elseif rand < longRouteProb && ~isempty(longCands)
            pool = [singleCands, medCands, longCands];
        else
            pool = [singleCands, medCands];
        end

        if singleCount >= maxSingleInt
            pool = pool(arrayfun(@(s) numel(s.ints) > 1, pool));
        end

        % Soft entrance cap
        validMask = arrayfun(@(s) entranceLoad(s.entrance) < maxPerEntrance, pool);
        if any(validMask), pool = pool(validMask); end

        % Hard intersection cap
        if isfinite(maxPerInt)
            intMask = arrayfun(@(s) all(totalIntLoad(s.ints) < maxPerInt), pool);
            if any(intMask), pool = pool(intMask); end
        end

        % Hard stage cap
        if isfinite(maxPerStage)
            stageMask = arrayfun(@(s) all( ...
                arrayfun(@(si) stageLoad(s.ints(si), si) < maxPerStage, 1:numel(s.ints)) ...
            ), pool);
            if any(stageMask), pool = pool(stageMask); end
        end

        if numel(pool) == 0
            error('Candidate pool became empty. Try increasing MaxPerEntrance or MaxPerIntersection.');
        end

        scores = inf(1, numel(pool));
        for c = 1:numel(pool)
            e    = pool(c).entrance;
            x    = pool(c).exit;
            ints = pool(c).ints;

            newEL = entranceLoad;    newEL(e) = newEL(e) + 1;
            newTL = totalIntLoad;    newSL = stageLoad;
            for s = 1:numel(ints)
                newTL(ints(s)) = newTL(ints(s)) + 1;
                newSL(ints(s), s) = newSL(ints(s), s) + 1;
            end

            totalScore = sum((newTL - mean(newTL)).^2);
            stageScore = 0;
            for s = 1:MAX_STG
                col = newSL(:, s).';
                if any(col), stageScore = stageScore + sum((col - mean(col)).^2); end
            end
            entranceScore = sum((newEL - mean(newEL)).^2);
            repScore      = countSameOD_5int(vehicleList, e, x);
            lenScore      = numel(ints) - 1;

            scores(c) = wTotal * totalScore + wStage * stageScore + ...
                        wEntrance * entranceScore + wSameOD * repScore + ...
                        wLong * lenScore;
        end

        if randomTieBreak
            [~, order] = sort(scores, 'ascend');
            topK = min(5, numel(order));
            chosenIdx = order(randi(topK));
        else
            [~, chosenIdx] = min(scores);
        end

        chosen = pool(chosenIdx);
        vehicleList(k).entrance = chosen.entrance;
        vehicleList(k).exit     = chosen.exit;
        vehicleList(k).ints     = chosen.ints;
        vehicleList(k).routeId  = chosen.routeId;
        vehicleList(k).subDir   = chosen.subDir;

        entranceLoad(chosen.entrance) = entranceLoad(chosen.entrance) + 1;
        for s = 1:numel(chosen.ints)
            totalIntLoad(chosen.ints(s)) = totalIntLoad(chosen.ints(s)) + 1;
            stageLoad(chosen.ints(s), s) = stageLoad(chosen.ints(s), s) + 1;
        end
        if numel(chosen.ints) == 1, singleCount = singleCount + 1; end
        if numel(chosen.ints) >= 3, longCount   = longCount   + 1; end
    end

    config = vehicleListToGroupedConfig_5int(vehicleList);

    stats = struct();
    stats.numVehicles      = numVehicles;
    stats.vehicleList      = vehicleList;
    stats.entranceLoad     = entranceLoad;
    stats.intersectionLoad = totalIntLoad;
    stats.stageLoad        = stageLoad;
    stats.singleIntCount   = singleCount;
    stats.longIntCount     = longCount;

    routeLenAll = arrayfun(@(v) numel(v.ints), vehicleList);
    fprintf('\nGenerated %d vehicles (5-int).\n', numVehicles);
    fprintf('Route length: 1-int=%d  2-int=%d  3-int=%d  4-int=%d\n', ...
        sum(routeLenAll==1), sum(routeLenAll==2), sum(routeLenAll==3), sum(routeLenAll>=4));
    fprintf('Int load [I1 I2 I3 I4 I5] = [%d %d %d %d %d]\n', ...
        totalIntLoad(1), totalIntLoad(2), totalIntLoad(3), totalIntLoad(4), totalIntLoad(5));
    fprintf('Entrance load [p1..p9] = [%s]\n', num2str(entranceLoad));
end


function candidates = buildCandidateList_5int(routeDict)
    candidates = struct('entrance',{},'exit',{},'ints',{},'routeId',{},'subDir',{});
    idx = 0;
    [nRow, nCol] = size(routeDict);
    for e = 1:nRow
        for x = 1:nCol
            if e == x, continue; end
            item = routeDict(e, x);
            if isstruct(item) && isfield(item,'int') && ~isempty(item.int)
                idx = idx + 1;
                candidates(idx).entrance = e;
                candidates(idx).exit     = x;
                candidates(idx).ints     = reshape(item.int, 1, []);
                candidates(idx).routeId  = item.routeId;
                candidates(idx).subDir   = item.subDir;
            end
        end
    end
end


function config = vehicleListToGroupedConfig_5int(vehicleList)
    entrances    = [vehicleList.entrance];
    usedEntrances = unique(entrances, 'stable');
    config = cell(numel(usedEntrances), 1);
    for i = 1:numel(usedEntrances)
        e   = usedEntrances(i);
        idx = find(entrances == e);
        exits = reshape([vehicleList(idx).exit], 1, []);
        config{i} = struct('entrance', e, 'exits', exits);
    end
end


function val = countSameOD_5int(vehicleList, e, x)
    val = 0;
    for i = 1:numel(vehicleList)
        if isempty(vehicleList(i).entrance), continue; end
        if vehicleList(i).entrance == e && vehicleList(i).exit == x
            val = val + 1;
        end
    end
end
