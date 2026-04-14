function intDB = buildIntSpaceDB(Map, C, numSpaces)
% buildIntSpaceDB  Build per-intersection space-usage lookup table from Map/C.
%
% INPUTS:
%   Map       : (S x R0) matrix
%               - Columns: routes, routeIdx = 1..R0
%               - Rows   : s-th occupied space (s = 1..S)
%               Map(s, r) = space index in {1,2,...,numSpaces}
%               Use 0 or NaN for "no s-th segment" on route r.
%
%   C         : (S x R0) matrix, same size as Map
%               C(s, r) = duration of occupying the space Map(s, r)
%               Use 0 or NaN for "no segment".
%
%   numSpaces : scalar, number of spaces in this intersection (e.g., 5)
%
% OUTPUT:
%   intDB : struct with fields
%       .numSpaces   : numSpaces
%       .numRoutes   : R0
%       .routeSpace  : 1xR0 cell, routeSpace{r} = [space_1 space_2 ...]
%       .routeDur    : 1xR0 cell, routeDur{r}   = [dur_1  dur_2  ...]
%       .routes(r)   : struct array, routes(r).space / .dur (same as cells)
%
% NOTES:
%   - Converts 0 to NaN internally for robust masking.
%   - Validates that space indices are integers in [1..numSpaces] and durations > 0.

    if nargin < 3 || isempty(numSpaces)
        error('buildIntSpaceDB:MissingNumSpaces', 'numSpaces is required (e.g., 5).');
    end
    if ~ismatrix(Map) || ~ismatrix(C)
        error('buildIntSpaceDB:BadInput', 'Map and C must be 2D matrices.');
    end
    if any(size(Map) ~= size(C))
        error('buildIntSpaceDB:SizeMismatch', 'Map and C must have the same size.');
    end

    % Convert 0 to NaN so we can mask cleanly
    Map(Map == 0) = NaN;
    C(C == 0)     = NaN;

    [S, R0] = size(Map);

    intDB = struct();
    intDB.numSpaces  = numSpaces;
    intDB.numRoutes  = R0;
    intDB.routeSpace = cell(1, R0);
    intDB.routeDur   = cell(1, R0);
    intDB.routes     = repmat(struct('space', [], 'dur', []), 1, R0);

    for r = 1:R0
        % s-th occupied space indices & durations on route r
        space_s = Map(:, r);   % size Sx1
        dur_s   = C(:, r);     % size Sx1

        mask = ~isnan(space_s);          % keep only existing segments
        space_seq = space_s(mask).';     % row vector [space_1 space_2 ...]
        dur_seq   = dur_s(mask).';       % row vector [dur_1 dur_2 ...]

        % --- validations ---
        if any(~isfinite(space_seq))
            error('buildIntSpaceDB:BadSpace', 'Route %d has non-finite space index.', r);
        end
        if any(space_seq < 1) || any(space_seq > numSpaces) || any(mod(space_seq,1) ~= 0)
            error('buildIntSpaceDB:SpaceOutOfRange', ...
                'Route %d has invalid space idx. Must be integer in [1,%d].', r, numSpaces);
        end
        if any(~isfinite(dur_seq)) || any(dur_seq <= 0)
            error('buildIntSpaceDB:BadDuration', ...
                'Route %d has invalid duration(s). Must be finite and > 0.', r);
        end
        if numel(space_seq) ~= numel(dur_seq)
            error('buildIntSpaceDB:LengthMismatch', ...
                'Route %d space/duration length mismatch after masking.', r);
        end

        intDB.routeSpace{r} = space_seq;
        intDB.routeDur{r}   = dur_seq;

        intDB.routes(r).space = space_seq;
        intDB.routes(r).dur   = dur_seq;
    end
end
