function [config, vehicleList, stats] = generateBalancedTrafficConfig(numVehicles, varargin)
%GENERATEBALANCEDTRAFFICCONFIG
% Generate grouped config in the SAME format as your manual input:
%
% config = {
%   struct('entrance', 1, 'exits', [7 3]),
%   struct('entrance', 4, 'exits', [1 3]),
%   ...
% };
%
% Main idea:
% Balance not only total vehicles through each intersection,
% but also whether that intersection is the 1st / 2nd / 3rd
% visited intersection along each vehicle's route.

    p = inputParser;
    addRequired(p, 'numVehicles', @(x)isnumeric(x) && isscalar(x) && x>=1);

    addParameter(p, 'MaxPerEntrance', ceil(numVehicles/8)+1, @(x)isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'EntrancePenalty', 0.30, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'StagePenalty', 1.00, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'TotalIntersectionPenalty', 0.35, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'SameODPenalty', 0.15, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'LongRoutePenalty', 0.25, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'LongRouteProb', 0.08, @(x)isnumeric(x) && isscalar(x) && x>=0 && x<=1);
    addParameter(p, 'RandomTieBreak', true, @(x)islogical(x) && isscalar(x));
    addParameter(p, 'Seed', [], @(x)isempty(x) || (isnumeric(x) && isscalar(x)));
    % ── new constraints ──
    addParameter(p, 'MaxSingleInt', 1, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'MinTripleInt', 1, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'MaxTripleInt', 2, @(x)isnumeric(x) && isscalar(x) && x>=0);
    addParameter(p, 'MaxPerIntersection', Inf, @(x)isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'MaxPerStage', Inf, @(x)isnumeric(x) && isscalar(x) && x>=1);
    parse(p, numVehicles, varargin{:});

    maxPerEntrance = p.Results.MaxPerEntrance;
    wEntrance      = p.Results.EntrancePenalty;
    wStage         = p.Results.StagePenalty;
    wTotal         = p.Results.TotalIntersectionPenalty;
    wSameOD        = p.Results.SameODPenalty;
    wLong          = p.Results.LongRoutePenalty;
    longRouteProb  = p.Results.LongRouteProb;
    randomTieBreak = p.Results.RandomTieBreak;
    seed           = p.Results.Seed;
    maxSingleInt   = p.Results.MaxSingleInt;
    minTripleInt   = p.Results.MinTripleInt;
    maxTripleInt   = p.Results.MaxTripleInt;
    maxPerInt      = p.Results.MaxPerIntersection;
    maxPerStage    = p.Results.MaxPerStage;

    if ~isempty(seed)
        rng(seed);
    end

    routeDict = generateTrafficSystem();
    candidates = buildCandidateList(routeDict);

    if isempty(candidates)
        error('No valid routes found in routeDict.');
    end

    % Separate by route length
    routeLens    = arrayfun(@(s) numel(s.ints), candidates);
    singleCands  = candidates(routeLens == 1);   % 1 intersection only
    medCands     = candidates(routeLens == 2);   % 2 intersections
    longCands    = candidates(routeLens >= 3);   % 3 intersections

    % State
    entranceLoad = zeros(1,8);
    totalIntLoad = zeros(1,4);
    stageLoad    = zeros(4,3);
    singleCount  = 0;   % tracks how many 1-int vehicles assigned so far
    tripleCount  = 0;   % tracks how many 3-int vehicles assigned so far

    vehicleList = repmat(struct( ...
        'entrance', [], ...
        'exit',     [], ...
        'ints',     [], ...
        'routeId',  [], ...
        'subDir',   []), 1, numVehicles);

    for k = 1:numVehicles

        remaining = numVehicles - k + 1;

        % Must-fill triple-int check: if we still need tripleCount < minTripleInt
        % and there are few vehicles left, force a long route
        mustFillTriple = (tripleCount < minTripleInt) && ...
                         (remaining <= (minTripleInt - tripleCount)) && ...
                         ~isempty(longCands);

        if mustFillTriple
            pool = longCands;
        elseif tripleCount >= maxTripleInt
            % Already hit max triple — only short/medium routes
            pool = [singleCands, medCands];
        elseif rand < longRouteProb && ~isempty(longCands)
            pool = [singleCands, medCands, longCands];
        else
            pool = [singleCands, medCands];
        end

        % Hard cap on 1-int (single) routes
        if singleCount >= maxSingleInt
            pool = pool(arrayfun(@(s) numel(s.ints) > 1, pool));
        end

        % Soft entrance cap
        validMask = arrayfun(@(s) entranceLoad(s.entrance) < maxPerEntrance, pool);
        if any(validMask)
            pool = pool(validMask);
        end

        % Hard intersection cap — reject any route that would push ANY intersection over limit
        if isfinite(maxPerInt)
            intMask = arrayfun(@(s) all(totalIntLoad(s.ints) < maxPerInt), pool);
            if any(intMask)
                pool = pool(intMask);
            end
        end

        % Hard stage cap — reject routes that would overload any (intersection, stage) cell
        % This prevents several vehicles from all clustering in the same corner as 1st stop
        if isfinite(maxPerStage)
            stageMask = arrayfun(@(s) all( ...
                arrayfun(@(si) stageLoad(s.ints(si), si) < maxPerStage, 1:numel(s.ints)) ...
            ), pool);
            if any(stageMask)
                pool = pool(stageMask);
            end
        end

        nPool = numel(pool);
        if nPool == 0
            error('Candidate pool became empty. Try increasing MaxPerEntrance or MaxPerIntersection.');
        end

        scores = inf(1, nPool);

        for c = 1:nPool
            e    = pool(c).entrance;
            x    = pool(c).exit;
            ints = pool(c).ints;

            newEntranceLoad = entranceLoad;
            newEntranceLoad(e) = newEntranceLoad(e) + 1;

            newTotalIntLoad = totalIntLoad;
            newStageLoad    = stageLoad;

            for s = 1:numel(ints)
                intId = ints(s);
                newTotalIntLoad(intId) = newTotalIntLoad(intId) + 1;
                newStageLoad(intId, s) = newStageLoad(intId, s) + 1;
            end

            % 1) balance total vehicles through each intersection
            totalScore = sum((newTotalIntLoad - mean(newTotalIntLoad)).^2);

            % 2) balance stage-wise loads:
            % for each stage, intersections should have similar counts
            stageScore = 0;
            for s = 1:3
                col = newStageLoad(:,s).';
                if any(col)
                    stageScore = stageScore + sum((col - mean(col)).^2);
                end
            end

            % 3) entrance usage balance
            entranceScore = sum((newEntranceLoad - mean(newEntranceLoad)).^2);

            % 4) avoid repeated exact same OD
            repScore = countSameOD(vehicleList, e, x);

            % 5) mild preference for short routes
            lenScore = numel(ints) - 1;

            scores(c) = ...
                wTotal    * totalScore + ...
                wStage    * stageScore + ...
                wEntrance * entranceScore + ...
                wSameOD   * repScore + ...
                wLong     * lenScore;
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
            intId = chosen.ints(s);
            totalIntLoad(intId) = totalIntLoad(intId) + 1;
            stageLoad(intId, s) = stageLoad(intId, s) + 1;
        end
        if numel(chosen.ints) == 1, singleCount = singleCount + 1; end
        if numel(chosen.ints) >= 3, tripleCount = tripleCount + 1; end
    end

    config = vehicleListToGroupedConfig(vehicleList);

    stats = struct();
    stats.numVehicles      = numVehicles;
    stats.vehicleList      = vehicleList;
    stats.entranceLoad     = entranceLoad;
    stats.intersectionLoad = totalIntLoad;
    stats.stageLoad        = stageLoad;
    stats.singleIntCount   = singleCount;
    stats.tripleIntCount   = tripleCount;

    routeLenAll = arrayfun(@(v) numel(v.ints), vehicleList);
    fprintf('\nGenerated %d vehicles.\n', numVehicles);
    fprintf('Route length distribution: 1-int=%d  2-int=%d  3-int=%d\n', ...
        sum(routeLenAll==1), sum(routeLenAll==2), sum(routeLenAll>=3));
    fprintf('Intersection load [I1 I2 I3 I4] = [%d %d %d %d]\n', ...
        totalIntLoad(1), totalIntLoad(2), totalIntLoad(3), totalIntLoad(4));
    fprintf('Entrance load [E1..E8] = [%s]\n', num2str(entranceLoad));
    fprintf('Stage load matrix (rows=I1..I4, cols=1st/2nd/3rd):\n');
    disp(stageLoad);
end


function candidates = buildCandidateList(routeDict)
    candidates = struct('entrance',{},'exit',{},'ints',{},'routeId',{},'subDir',{});
    idx = 0;

    [nRow, nCol] = size(routeDict);
    for e = 1:nRow
        for x = 1:nCol
            if e == x
                continue;
            end

            item = routeDict(e,x);

            if isstruct(item) && isfield(item,'int') && ~isempty(item.int)
                idx = idx + 1;
                candidates(idx).entrance = e;
                candidates(idx).exit     = x;
                candidates(idx).ints     = reshape(item.int, 1, []);
                if isfield(item,'routeId')
                    candidates(idx).routeId = item.routeId;
                else
                    candidates(idx).routeId = [];
                end
                if isfield(item,'subDir')
                    candidates(idx).subDir = item.subDir;
                else
                    candidates(idx).subDir = [];
                end
            end
        end
    end
end


function config = vehicleListToGroupedConfig(vehicleList)
    entrances = [vehicleList.entrance];
    usedEntrances = unique(entrances, 'stable');

    config = cell(numel(usedEntrances), 1);

    for i = 1:numel(usedEntrances)
        e = usedEntrances(i);
        idx = find(entrances == e);
        exits = [vehicleList(idx).exit];
        exits = reshape(exits, 1, []);
        config{i} = struct('entrance', e, 'exits', exits);
    end
end


function val = countSameOD(vehicleList, e, x)
    val = 0;
    for i = 1:numel(vehicleList)
        if isempty(vehicleList(i).entrance)
            continue;
        end
        if vehicleList(i).entrance == e && vehicleList(i).exit == x
            val = val + 1;
        end
    end
end