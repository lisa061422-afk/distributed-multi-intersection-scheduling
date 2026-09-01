function export_panel_grid_png(panelHandles, outputFile)
%EXPORT_PANEL_GRID_PNG Export five panels as a 2-by-3 map layout.
%   The lower-right tile remains blank, matching the five-intersection map.
%   Exporting panels independently avoids whole-window capture and remains
%   robust when OS text scaling makes MATLAB figures scrollable.

tiles = cell(1, numel(panelHandles));
tileHeight = 0;
tileWidth = 0;

for idx = 1:numel(panelHandles)
    if ~isgraphics(panelHandles(idx)), continue; end
    tempFile = [tempname '.png'];
    exportgraphics(panelHandles(idx), tempFile, ...
        'Resolution', 150, 'BackgroundColor', 'white');
    tiles{idx} = imread(tempFile);
    delete(tempFile);
    tileHeight = max(tileHeight, size(tiles{idx}, 1));
    tileWidth = max(tileWidth, size(tiles{idx}, 2));
end

if tileHeight == 0 || tileWidth == 0
    error('No valid five-intersection panels were available for export.');
end

gap = 24;
canvas = uint8(255 * ones(2 * tileHeight + gap, ...
    3 * tileWidth + 2 * gap, 3));

for idx = 1:numel(tiles)
    tile = tiles{idx};
    if isempty(tile), continue; end
    if size(tile, 3) == 1
        tile = repmat(tile, 1, 1, 3);
    end
    if idx <= 3
        row = 1;
        col = idx;
    else
        row = 2;
        col = idx - 3;
    end
    h = size(tile, 1);
    w = size(tile, 2);
    r0 = (row - 1) * (tileHeight + gap) + 1 + floor((tileHeight - h) / 2);
    c0 = (col - 1) * (tileWidth + gap) + 1 + floor((tileWidth - w) / 2);
    canvas(r0:r0+h-1, c0:c0+w-1, :) = tile(:,:,1:3);
end

imwrite(canvas, outputFile);
end
