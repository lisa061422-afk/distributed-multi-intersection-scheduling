function fig = plot_decision_tree_panel(LocalTreeCache, save_path)
% plot_decision_tree_panel  2×2 panel of pruned decision trees for 4 intersections.
%
% Only "branching" nodes (those with >=2 children in the full tree) and their
% direct children are displayed. Single-child chains are compressed into direct
% edges so the diagram stays readable. The optimal path is highlighted in red.
%
% INPUTS
%   LocalTreeCache  cell(9,1) — LocalTreeCache{1..4} used (intersection agents)
%   save_path       (optional) full path for PNG output (e.g. 'out/tree.png')
%
% OUTPUT
%   fig   figure handle

fig = figure('Color', 'w', 'Position', [50 50 1600 950], ...
    'Name', 'Decision Trees — C-ADMM');

panelPos = {
    [0.04 0.54 0.44 0.42],
    [0.52 0.54 0.44 0.42],
    [0.04 0.04 0.44 0.42],
    [0.52 0.04 0.44 0.42]
};

for agent_i = 1:4
    cache = LocalTreeCache{agent_i};
    ax = axes('Parent', fig, 'Position', panelPos{agent_i}); %#ok<LAXES>

    if isempty(cache) || ~isstruct(cache) || ~isfield(cache, 'NODES')
        text(ax, 0.5, 0.5, sprintf('Merging Zone %d — no data', agent_i), ...
            'HorizontalAlignment', 'center', 'Units', 'normalized', 'FontSize', 10);
        axis(ax, 'off');
        continue;
    end

    NODES     = cache.NODES;
    Path      = cache.Path;      % row vector of node indices: root→leaf
    num_nodes = size(NODES, 1);

    % ── 1. Build children map ───────────────────────────────────────────────
    children = cell(num_nodes, 1);
    for i = 1:num_nodes
        p = NODES{i}{7};          % 7th field = parent node index
        if p > 0 && p <= num_nodes
            children{p} = [children{p}, i];
        end
    end

    % ── 2. Identify display nodes ────────────────────────────────────────────
    %  Keep: root  +  any node with ≥2 children  +  direct children of such
    %  nodes  +  leaf nodes
    is_branch  = false(num_nodes, 1);
    is_leaf    = false(num_nodes, 1);
    for i = 1:num_nodes
        nc = numel(children{i});
        if nc >= 2,   is_branch(i) = true; end
        if nc == 0,   is_leaf(i)   = true; end
    end

    is_display    = false(num_nodes, 1);
    is_display(1) = true;            % root always shown
    for i = 1:num_nodes
        if is_branch(i)
            is_display(i)            = true;
            is_display(children{i})  = true;   % show all direct children
        end
        if is_leaf(i)
            is_display(i) = true;
        end
    end

    display_nodes = find(is_display);
    nd = numel(display_nodes);

    if nd < 2
        text(ax, 0.5, 0.5, ...
            sprintf('Merging Zone %d\n(no branching — trivial schedule)', agent_i), ...
            'HorizontalAlignment', 'center', 'Units', 'normalized', 'FontSize', 9);
        title(ax, sprintf('Merging Zone %d', agent_i), 'FontSize', 10, 'FontWeight', 'bold');
        axis(ax, 'off');
        continue;
    end

    % Map old (NODES) index → compressed (display) index
    old2new = zeros(num_nodes, 1);
    for k = 1:nd
        old2new(display_nodes(k)) = k;
    end

    % ── 3. Compressed edges ──────────────────────────────────────────────────
    %  For each display node (excluding root), walk parent pointers upward until
    %  we reach another display node. That ancestor becomes the edge source.
    src_list = [];
    dst_list = [];
    for k = 1:nd
        orig = display_nodes(k);
        if orig == 1, continue; end     % root has no parent
        p = NODES{orig}{7};
        while p > 0 && ~is_display(p)
            p = NODES{p}{7};
        end
        if p > 0 && is_display(p)
            src_list = [src_list, old2new(p)];   %#ok<AGROW>
            dst_list = [dst_list, old2new(orig)]; %#ok<AGROW>
        end
    end

    if isempty(src_list)
        title(ax, sprintf('Merging Zone %d — empty graph', agent_i), 'FontSize', 10);
        axis(ax, 'off');
        continue;
    end

    % ── 4. Node labels ───────────────────────────────────────────────────────
    node_labels = strings(nd, 1);
    for k = 1:nd
        orig   = display_nodes(k);
        tw_val = NODES{orig}{5};
        f_val  = NODES{orig}{12};
        node_labels(k) = sprintf('%d: tw=%.1f f=%.2f', orig, tw_val, f_val);
    end

    % ── 5. Build and plot digraph ─────────────────────────────────────────────
    G = digraph(src_list, dst_list, [], nd);

    try
        h = plot(G, 'Layout', 'layered', 'Parent', ax);
    catch
        try
            h = plot(G, 'Layout', 'auto', 'Parent', ax);
        catch
            h = plot(G, 'Parent', ax);
        end
    end

    h.NodeLabel   = {};
    h.NodeColor   = [0.22 0.50 0.80];
    h.MarkerSize  = 5;
    h.EdgeColor   = [0.55 0.55 0.55];
    h.LineWidth   = 1.2;

    % Spread nodes for readability
    h.XData = h.XData * 2.8;
    h.YData = h.YData * 2.0;

    % Text labels (offset above node)
    x_data = h.XData;
    y_data = h.YData;
    y_range = max(y_data) - min(y_data);
    offset  = max(0.18, y_range * 0.07);
    for k = 1:nd
        text(ax, x_data(k), y_data(k) + offset, node_labels(k), ...
            'FontSize', 6.5, 'HorizontalAlignment', 'center', ...
            'Color', [0.12 0.12 0.12], 'Interpreter', 'none');
    end

    % ── 6. Highlight optimal path (in display) ──────────────────────────────
    if ~isempty(Path) && numel(Path) >= 1
        % Keep only path nodes that are in the display set
        valid_mask = (Path >= 1) & (Path <= num_nodes) & is_display(Path)';
        path_disp  = Path(valid_mask);
        path_new   = old2new(path_disp);
        path_new   = path_new(path_new > 0);

        if numel(path_new) >= 2
            highlight(h, path_new, ...
                'NodeColor', [0.88 0.18 0.08], ...
                'EdgeColor', [0.88 0.18 0.08], ...
                'LineWidth', 2.5);
        elseif numel(path_new) == 1
            highlight(h, path_new, 'NodeColor', [0.88 0.18 0.08]);
        end
    end

    title(ax, sprintf('Merging Zone %d  —  Decision Tree  (branching nodes)', agent_i), ...
        'FontSize', 9, 'FontWeight', 'bold');
    axis(ax, 'off');
    box(ax, 'off');
end

% Super-title
annotation(fig, 'textbox', [0 0.95 1 0.05], ...
    'String', 'Local Decision Trees — Distributed C-ADMM (last ADMM iteration)', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 11, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none');

drawnow;

if nargin >= 2 && ~isempty(save_path)
    exportgraphics(fig, save_path, 'Resolution', 200);
    fprintf('Decision tree panel saved: %s\n', save_path);
end
end
