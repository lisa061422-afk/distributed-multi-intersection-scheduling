function [min_f, minIndex] = f_min2(NODES, OPEN)

best_key = [Inf, Inf, Inf];
min_f = Inf;
minIndex = OPEN(1);

for k = 1:numel(OPEN)
    idx = OPEN(k); 

    f_val = NODES{idx}{12};
    tw_val = NODES{idx}{5};

    % key = [f, tw, idx]
    key = [f_val, tw_val, idx];

    if isequal(best_key, [Inf, Inf, Inf]) || ...
       key(1) < best_key(1) || ...
       (abs(key(1)-best_key(1)) <= 1e-10 && key(2) < best_key(2)) || ...
       (abs(key(1)-best_key(1)) <= 1e-10 && abs(key(2)-best_key(2)) <= 1e-10 && key(3) < best_key(3))

        best_key = key;
        min_f = f_val;
        minIndex = idx;
    end
end

end