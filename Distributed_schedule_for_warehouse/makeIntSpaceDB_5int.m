function IntSpaceDB = makeIntSpaceDB_5int()
% makeIntSpaceDB_5int  Same local M1-M5 structure as warehouse, extended to 5 intersections.
% I5 uses the identical space layout — only road connections differ.

    IntSpaceDB = makeIntSpaceDB();          % builds I1..I4 (cell 1x4)
    IntSpaceDB{end+1} = IntSpaceDB{1};     % I5 = same as I1..I4
end
