function entranceOffset = generateSeparatedEntranceOffsets(numEntrances, minGap, maxGap, seed)

    if nargin >= 4 && ~isempty(seed)
        rng(seed);
    end

    entranceOffset = zeros(1, numEntrances);

    % first entrance starts near 0
    entranceOffset(1) = rand();

    for e = 2:numEntrances
        gap = minGap + (maxGap - minGap) * rand();
        entranceOffset(e) = entranceOffset(e-1) + gap;
    end

    % randomly assign these offsets to the physical entrances
    perm = randperm(numEntrances);
    entranceOffset = entranceOffset(perm);
end