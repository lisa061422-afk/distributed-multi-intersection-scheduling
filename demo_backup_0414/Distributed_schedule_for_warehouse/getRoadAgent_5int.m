function id = getRoadAgent_5int(i, j)
% getRoadAgent_5int  Road agent ID for the 5-intersection traffic map.
%
% Topology:
%   I1(1) — I2(2) — I5(5)
%   |         |
%   I3(3) — I4(4)
%
% Road agents:
%   6 : I1 ↔ I2
%   7 : I2 ↔ I5
%   8 : I1 ↔ I3
%   9 : I2 ↔ I4
%  10 : I3 ↔ I4

    if     (i==1 && j==2) || (i==2 && j==1),  id = 6;
    elseif (i==2 && j==5) || (i==5 && j==2),  id = 7;
    elseif (i==1 && j==3) || (i==3 && j==1),  id = 8;
    elseif (i==2 && j==4) || (i==4 && j==2),  id = 9;
    elseif (i==3 && j==4) || (i==4 && j==3),  id = 10;
    else
        error('getRoadAgent_5int: unknown road between intersection %d and %d', i, j);
    end
end
