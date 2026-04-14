function [alpha_tilde, initial_position, init_p_vehi_1, entranceOffset] = ...
    generateAlphaTildeByEntrance(config, detect_range, v_max, d1, W, headway, randWindow, seed)

    if nargin >= 8 && ~isempty(seed)
        rng(seed);
    end

    N = numel(config);
    alpha_tilde = cell(N,1);
    initial_position = zeros(N,1);
    init_p_vehi_1 = zeros(N,1);

    numEntrances = 8;
    entranceOffset = randWindow * rand(1, numEntrances);

    % group vehicles by entrance
    entranceVeh = cell(1, numEntrances);
    for n = 1:N
        e = config{n}.entrance;
        entranceVeh{e} = [entranceVeh{e}, n];
    end

    for e = 1:numEntrances
        vehs = entranceVeh{e};
        if isempty(vehs)
            continue;
        end

        % sort by entryIndex
        entryIdx = zeros(1, numel(vehs));
        for k = 1:numel(vehs)
            entryIdx(k) = config{vehs(k)}.entryIndex;
        end
        [~, ord] = sort(entryIdx);
        vehs = vehs(ord);

        % first vehicle of this entrance
        n0 = vehs(1);
        base0 = (detect_range(n0)/2 - W/2) / v_max(n0) + d1(n0);
        alpha_tilde{n0} = base0 + entranceOffset(e);

        % following vehicles: exact headway from previous vehicle
        for k = 2:numel(vehs)
            n = vehs(k);
            prev = vehs(k-1);
            alpha_tilde{n} = alpha_tilde{prev} + headway;
        end
    end

    % back-compute initial positions
    for n = 1:N
        t_arr = alpha_tilde{n}(1);
        init_p_vehi_1(n) = -(detect_range(n)/2 - W/2 + t_arr * v_max(n));
        initial_position(n,1) = init_p_vehi_1(n);
    end
end