function id = getRoadAgent(i, j)
    if (i == 1 && j == 4) || (i == 4 && j == 1)
        id = 8;
    elseif (i == 2 && j == 1) || (i == 1 && j == 2)
        id = 5;
    elseif (i == 3 && j == 2) || (i == 2 && j == 3)
        id = 6;
    elseif (i == 4 && j == 3) || (i == 3 && j == 4)
        id = 7;
    else
        error('Unknown road from %d to %d', i, j);
    end
end
