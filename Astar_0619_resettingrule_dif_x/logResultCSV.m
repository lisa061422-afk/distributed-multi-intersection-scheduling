function logResultCSV(cfg, opt_cost, n_leaves, n_nodes, elapsed, timed_out, csv_path)
% logResultCSV  Accumulate run results in a 2D comparison CSV.
%
%   Rows    = metrics (ON_cost, OFF_cost, ON_nLeaves, ...)
%   Columns = one per unique (routeAssignment, d1) case
%
%   Each run updates the appropriate cell; new cases add a new column.
%   Timed-out costs are marked with *.

if nargin < 7 || isempty(csv_path)
    csv_path = fullfile(fileparts(mfilename('fullpath')), 'results_comparison.csv');
end

%% Case key — no spaces, no commas (safe as CSV column header)
r_str    = strjoin(arrayfun(@num2str, cfg.routeAssignment, 'UniformOutput', false), '-');
d_str    = strjoin(arrayfun(@num2str, cfg.d1, 'UniformOutput', false), '-');
case_key = sprintf('R[%s]_d1[%s]', r_str, d_str);

%% Fixed row labels (row 1 = header, rows 2-9 = metrics)
ROWS = {'Metric','ON_cost','OFF_cost','ON_nLeaves','OFF_nLeaves',...
        'ON_nNodes','OFF_nNodes','ON_time_s','OFF_time_s'};
N_ROWS = numel(ROWS);   % 9

%% Which data rows this run updates
if cfg.useWeakRule
    ir_cost = 2;  ir_leaf = 4;  ir_node = 6;  ir_time = 8;
else
    ir_cost = 3;  ir_leaf = 5;  ir_node = 7;  ir_time = 9;
end

%% Read existing CSV or start fresh
if exist(csv_path, 'file')
    mat = readCSVtoCell(csv_path);
    % Ensure enough rows
    while size(mat,1) < N_ROWS
        mat{end+1, 1} = ''; %#ok<AGROW>
    end
    % Fill in row labels if missing
    for ri = 1:N_ROWS
        if isempty(mat{ri,1}),  mat{ri,1} = ROWS{ri};  end
    end
    % Find column for this case (search header row, cols 2+)
    col_idx = 0;
    for ci = 2:size(mat,2)
        if strcmp(mat{1,ci}, case_key),  col_idx = ci;  break;  end
    end
    if col_idx == 0
        col_idx = size(mat,2) + 1;
        mat{1, col_idx} = case_key;
    end
else
    % Fresh table: N_ROWS × 2
    mat = repmat({''}, N_ROWS, 2);
    for ri = 1:N_ROWS,  mat{ri,1} = ROWS{ri};  end
    col_idx = 2;
    mat{1, col_idx} = case_key;
end

%% Format values
if isnan(opt_cost)
    cost_str = 'no_solution';
elseif timed_out
    cost_str = sprintf('%.4f*', opt_cost);
else
    cost_str = sprintf('%.4f', opt_cost);
end

mat{ir_cost, col_idx} = cost_str;
mat{ir_leaf, col_idx} = num2str(n_leaves);
mat{ir_node, col_idx} = num2str(n_nodes);
mat{ir_time, col_idx} = sprintf('%.2f', elapsed);

%% Write back
writeCSVfromCell(mat, csv_path);
fprintf('CSV updated: %s  [%s  WeakRule=%d  cost=%s]\n', ...
    csv_path, case_key, cfg.useWeakRule, cost_str);
end


% =========================================================================
function mat = readCSVtoCell(csv_path)
% Read a CSV file into a cell matrix of strings. Does NOT handle
% quoted fields that span multiple lines, but quoted commas are handled.
fid = fopen(csv_path, 'r', 'n', 'UTF-8');
lines = {};
while ~feof(fid)
    ln = fgetl(fid);
    if ischar(ln),  lines{end+1} = ln;  end %#ok<AGROW>
end
fclose(fid);

if isempty(lines)
    mat = {};
    return;
end

n_rows = numel(lines);
parsed = cell(n_rows, 1);
max_cols = 0;
for ri = 1:n_rows
    parsed{ri} = splitCSVline(lines{ri});
    max_cols = max(max_cols, numel(parsed{ri}));
end

mat = repmat({''}, n_rows, max(max_cols,1));
for ri = 1:n_rows
    for ci = 1:numel(parsed{ri})
        mat{ri,ci} = strtrim(parsed{ri}{ci});
    end
end
end


% =========================================================================
function parts = splitCSVline(line)
% Split a CSV line respecting double-quoted fields.
parts = {};
cur   = '';
in_q  = false;
for k = 1:numel(line)
    c = line(k);
    if c == '"'
        in_q = ~in_q;
    elseif c == ',' && ~in_q
        parts{end+1} = cur; %#ok<AGROW>
        cur = '';
    else
        cur(end+1) = c; %#ok<AGROW>
    end
end
parts{end+1} = cur;
end


% =========================================================================
function writeCSVfromCell(mat, csv_path)
% Write cell matrix to CSV, quoting fields that contain commas.
fid = fopen(csv_path, 'w', 'n', 'UTF-8');
if fid < 0
    % File locked — try timestamped backup
    backup = strrep(csv_path, '.csv', ...
        sprintf('_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
    fid = fopen(backup, 'w', 'n', 'UTF-8');
    if fid < 0
        warning('logResultCSV: cannot write CSV — result NOT saved.');
        return;
    end
    warning('logResultCSV: original file locked (open in Excel?). Saved to:\n  %s', backup);
end

[nr, nc] = size(mat);
for ri = 1:nr
    for ci = 1:nc
        if ci > 1,  fprintf(fid, ',');  end
        val = mat{ri,ci};
        if ~ischar(val),  val = '';  end
        if any(val == ',')
            fprintf(fid, '"%s"', val);
        else
            fprintf(fid, '%s', val);
        end
    end
    fprintf(fid, '\n');
end
fclose(fid);
end
