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

    if ~isempty(seed)
        rng(seed);
    end

    routeDict = generateTrafficSystem();
    candidates = buildCandidateList(routeDict);

    if isempty(candidates)
        error('No valid routes found in routeDict.');
    end

    % Prefer short routes most of the time
    routeLens  = arrayfun(@(s) numel(s.ints), candidates);
    shortCands = candidates(routeLens <= 2);
    longCands  = candidates(routeLens >= 3);

    % State
    entranceLoad = zeros(1,8);     % 1x8
    totalIntLoad = zeros(1,4);     % 1x4
    stageLoad    = zeros(4,3);     % rows = intersections 1..4, cols = stage 1..3

    vehicleList = repmat(struct( ...
        'entrance', [], ...
        'exit',     [], ...
        'ints',     [], ...
        'routeId',  [], ...
        'subDir',   []), 1, numVehicles);

    for k = 1:numVehicles

        if rand < longRouteProb && ~isempty(longCands)
            pool = [shortCands, longCands];
        else
            pool = shortCands;
        end

        % soft entrance cap
        validMask = arrayfun(@(s) entranceLoad(s.entrance) < maxPerEntrance, pool);
        if any(validMask)
            pool = pool(validMask);
        end

        nPool = numel(pool);
        if nPool == 0
            error('Candidate pool became empty. Try increasing MaxPerEntrance.');
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
    end

    config = vehicleListToGroupedConfig(vehicleList);

    stats = struct();
    stats.numVehicles      = numVehicles;
    stats.vehicleList      = vehicleList;
    stats.entranceLoad     = entranceLoad;
    stats.intersectionLoad = totalIntLoad;
    stats.stageLoad        = stageLoad;

    fprintf('\nGenerated %d vehicles.\n', numVehicles);
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