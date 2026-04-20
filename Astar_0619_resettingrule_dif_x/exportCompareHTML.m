function exportCompareHTML(results, caseNames, out_path)
% exportCompareHTML  Generate comparison.html from batch run results.
%
%   results   : struct array from runBatch.m
%   caseNames : Nx1 cell of case name strings (order for columns)
%   out_path  : output file path

if nargin < 3
    out_path = fullfile(fileparts(mfilename('fullpath')), 'comparison.html');
end

nCases = numel(caseNames);

% Build lookup: caseName × WeakRule → result index
lookup = containers.Map();
for ri = 1:numel(results)
    key = sprintf('%s|%d', results(ri).caseName, results(ri).useWeak);
    lookup(key) = ri;
end

% Collect data arrays for chart
cost_on  = nan(1, nCases);
cost_off = nan(1, nCases);
for ci = 1:nCases
    cn = caseNames{ci};
    k_on  = sprintf('%s|1', cn);
    k_off = sprintf('%s|0', cn);
    if isKey(lookup, k_on),  cost_on(ci)  = results(lookup(k_on)).optCost;  end
    if isKey(lookup, k_off), cost_off(ci) = results(lookup(k_off)).optCost; end
end

fid = fopen(out_path, 'w', 'n', 'UTF-8');
if fid < 0,  error('Cannot write: %s', out_path);  end

%% HTML head
fprintf(fid, '<!DOCTYPE html>\n<html lang="en">\n<head>\n');
fprintf(fid, '<meta charset="UTF-8">\n');
fprintf(fid, '<title>A* WeakRule Comparison</title>\n');
fprintf(fid, '<style>\n');
fprintf(fid, '*{box-sizing:border-box;margin:0;padding:0}\n');
fprintf(fid, 'html,body{background:radial-gradient(circle at top,#102133 0%%,#08111b 55%%,#04090f 100%%);');
fprintf(fid, 'color:#eef4ff;font-family:Inter,system-ui,sans-serif;min-height:100vh;padding:24px}\n');
fprintf(fid, 'h1{font-size:20px;font-weight:700;color:#79caff;margin-bottom:6px}\n');
fprintf(fid, '.subtitle{font-size:12px;color:#6b7280;margin-bottom:24px}\n');
fprintf(fid, '.section{background:rgba(12,24,38,.85);border:1px solid rgba(255,255,255,.1);');
fprintf(fid, 'border-radius:10px;padding:20px;margin-bottom:22px}\n');
fprintf(fid, 'h2{font-size:14px;font-weight:600;color:#93c5fd;margin-bottom:14px;letter-spacing:.5px}\n');
% Table styles
fprintf(fid, 'table{border-collapse:collapse;width:100%%;font-size:12px}\n');
fprintf(fid, 'th{background:#0d1d2e;color:#9ca3af;font-weight:600;padding:8px 12px;');
fprintf(fid, 'text-align:center;border:1px solid #1e3a5f}\n');
fprintf(fid, 'th.row-hdr{text-align:left;min-width:120px}\n');
fprintf(fid, 'td{padding:7px 12px;text-align:center;border:1px solid #1a2a3a}\n');
fprintf(fid, 'td.label{text-align:left;color:#9ca3af;font-size:11px}\n');
fprintf(fid, '.on-row td{background:#0a1e10}\n');
fprintf(fid, '.off-row td{background:#1a0e0a}\n');
fprintf(fid, '.cost{font-size:14px;font-weight:700}\n');
fprintf(fid, '.on-cost{color:#22c55e}\n');
fprintf(fid, '.off-cost{color:#f97316}\n');
fprintf(fid, '.better{outline:2px solid #22c55e;outline-offset:-2px}\n');
fprintf(fid, '.sub{font-size:10px;color:#6b7280;display:block;margin-top:2px}\n');
fprintf(fid, '.timeout{color:#fbbf24;font-size:10px}\n');
fprintf(fid, '.na{color:#374151}\n');
fprintf(fid, 'a{color:#79caff;text-decoration:none}\n');
fprintf(fid, 'a:hover{text-decoration:underline}\n');
fprintf(fid, '</style>\n</head>\n<body>\n');

fprintf(fid, '<h1>A* Intersection Scheduler — WeakRule Comparison</h1>\n');
fprintf(fid, '<div class="subtitle">Generated %s &nbsp;|&nbsp; %d cases &nbsp;|&nbsp; ', ...
    datestr(now, 'yyyy-mm-dd HH:MM'), nCases);
fprintf(fid, 'timeout = %ds</div>\n', results(1).elapsed > 0 && isfield(results,'timedOut'));

%% Results table
fprintf(fid, '<div class="section">\n');
fprintf(fid, '<h2>OPTIMAL COST COMPARISON</h2>\n');
fprintf(fid, '<table>\n<tr><th class="row-hdr">WeakRule</th>\n');
for ci = 1:nCases
    fprintf(fid, '<th>%s</th>\n', caseNames{ci});
end
fprintf(fid, '</tr>\n');

for useWeak = [true, false]
    rowClass = 'on-row';  ruleStr = 'ON';  costClass = 'on-cost';
    if ~useWeak,  rowClass = 'off-row';  ruleStr = 'OFF';  costClass = 'off-cost';  end

    fprintf(fid, '<tr class="%s"><td class="label"><b>WeakRule %s</b></td>\n', rowClass, ruleStr);
    for ci = 1:nCases
        cn  = caseNames{ci};
        key = sprintf('%s|%d', cn, useWeak);
        if isKey(lookup, key)
            r = results(lookup(key));
            % Check if this is better (lower cost) than the other rule
            other_key = sprintf('%s|%d', cn, ~useWeak);
            is_better = false;
            if isKey(lookup, other_key)
                r2 = results(lookup(other_key));
                if ~isnan(r.optCost) && ~isnan(r2.optCost) && r.optCost < r2.optCost - 1e-6
                    is_better = true;
                end
            end
            better_cls = '';  if is_better, better_cls = ' better'; end

            if isnan(r.optCost)
                cost_str = '<span class="na">—</span>';
                sub_str  = '<span class="sub timeout">no solution</span>';
            else
                to_str = '';  if r.timedOut, to_str = '*'; end
                cost_str = sprintf('<span class="cost %s">%.4f%s</span>', costClass, r.optCost, to_str);
                sub_str  = sprintf('<span class="sub">%d leaves &nbsp; %.1fs</span>', r.nLeaves, r.elapsed);
            end

            link = '';
            if ~isempty(r.htmlFile)
                link = sprintf('&nbsp;<a href="%s" target="_blank">[demo]</a>', r.htmlFile);
            end
            fprintf(fid, '<td class="%s">%s%s%s</td>\n', better_cls, cost_str, sub_str, link);
        else
            fprintf(fid, '<td class="na">—</td>\n');
        end
    end
    fprintf(fid, '</tr>\n');
end

% Extra rows: nNodes comparison
fprintf(fid, '<tr><td class="label" style="color:#6b7280">nodes (ON)</td>\n');
for ci = 1:nCases
    key = sprintf('%s|1', caseNames{ci});
    if isKey(lookup, key)
        fprintf(fid, '<td><span style="font-size:11px;color:#6b7280">%d</span></td>\n', ...
            results(lookup(key)).nNodes);
    else, fprintf(fid, '<td class="na">—</td>\n'); end
end
fprintf(fid, '</tr>\n');
fprintf(fid, '<tr><td class="label" style="color:#6b7280">nodes (OFF)</td>\n');
for ci = 1:nCases
    key = sprintf('%s|0', caseNames{ci});
    if isKey(lookup, key)
        fprintf(fid, '<td><span style="font-size:11px;color:#6b7280">%d</span></td>\n', ...
            results(lookup(key)).nNodes);
    else, fprintf(fid, '<td class="na">—</td>\n'); end
end
fprintf(fid, '</tr>\n');
fprintf(fid, '</table>\n');
fprintf(fid, '<p style="margin-top:8px;font-size:10px;color:#4b5563">');
fprintf(fid, '* = timed out (best partial result) &nbsp;&nbsp; ');
fprintf(fid, '<span style="outline:2px solid #22c55e;padding:1px 4px">lower cost</span> highlighted green</p>\n');
fprintf(fid, '</div>\n');

%% Bar chart
fprintf(fid, '<div class="section">\n');
fprintf(fid, '<h2>COST BAR CHART</h2>\n');
fprintf(fid, '<svg id="chart" style="width:100%%;display:block"></svg>\n');
fprintf(fid, '</div>\n');

%% Cost delta table
fprintf(fid, '<div class="section">\n');
fprintf(fid, '<h2>COST DIFFERENCE (OFF − ON) &nbsp; <span style="font-size:11px;color:#6b7280">positive = WeakRule reduces cost</span></h2>\n');
fprintf(fid, '<table>\n<tr><th class="row-hdr">Metric</th>\n');
for ci = 1:nCases
    fprintf(fid, '<th>%s</th>\n', caseNames{ci});
end
fprintf(fid, '</tr>\n');
fprintf(fid, '<tr><td class="label">OFF − ON cost</td>\n');
for ci = 1:nCases
    kon = sprintf('%s|1', caseNames{ci});
    kof = sprintf('%s|0', caseNames{ci});
    if isKey(lookup,kon) && isKey(lookup,kof)
        delta = results(lookup(kof)).optCost - results(lookup(kon)).optCost;
        if isnan(delta)
            fprintf(fid, '<td class="na">—</td>\n');
        else
            col = '#22c55e';  if delta < -1e-6, col = '#ef4444'; end
            fprintf(fid, '<td><span style="font-weight:700;color:%s">%+.4f</span></td>\n', col, delta);
        end
    else
        fprintf(fid, '<td class="na">—</td>\n');
    end
end
fprintf(fid, '</tr>\n');
fprintf(fid, '<tr><td class="label">nodes ON / OFF</td>\n');
for ci = 1:nCases
    kon = sprintf('%s|1', caseNames{ci});
    kof = sprintf('%s|0', caseNames{ci});
    if isKey(lookup,kon) && isKey(lookup,kof)
        fprintf(fid, '<td style="font-size:11px;color:#9ca3af">%d / %d</td>\n', ...
            results(lookup(kon)).nNodes, results(lookup(kof)).nNodes);
    else, fprintf(fid, '<td class="na">—</td>\n'); end
end
fprintf(fid, '</tr>\n');
fprintf(fid, '</table>\n</div>\n');

%% JS for bar chart
fprintf(fid, '<script>\n');
fprintf(fid, 'const LABELS=[');
for ci = 1:nCases
    if ci>1, fprintf(fid,','); end
    fprintf(fid,'"%s"', caseNames{ci});
end
fprintf(fid,'];\n');
fprintf(fid,'const CON=[');
for ci=1:nCases
    if ci>1, fprintf(fid,','); end
    if isnan(cost_on(ci)), fprintf(fid,'null'); else, fprintf(fid,'%.4f',cost_on(ci)); end
end
fprintf(fid,'];\n');
fprintf(fid,'const COFF=[');
for ci=1:nCases
    if ci>1, fprintf(fid,','); end
    if isnan(cost_off(ci)), fprintf(fid,'null'); else, fprintf(fid,'%.4f',cost_off(ci)); end
end
fprintf(fid,'];\n');

js = {
'(function(){'
'  const svg=document.getElementById("chart");'
'  const W=svg.parentElement.clientWidth-40||700;'
'  const H=220, padL=52,padR=16,padT=20,padB=40;'
'  const cW=Math.max(8,(W-padL-padR)/(LABELS.length*2+LABELS.length+1));'
'  const groupW=cW*2+4;'
'  const all=[...CON,...COFF].filter(v=>v!==null);'
'  if(!all.length)return;'
'  const maxV=Math.max(...all)*1.12, minV=0;'
'  const yScale=v=>(H-padT-padB)*(1-(v-minV)/(maxV-minV))+padT;'
'  let h=`<rect width="${W}" height="${H}" fill="transparent"/>`;'
'  // y gridlines'
'  for(let i=0;i<=4;i++){'
'    const v=minV+i*(maxV-minV)/4;'
'    const y=yScale(v);'
'    h+=`<line x1="${padL}" y1="${y}" x2="${W-padR}" y2="${y}" stroke="#1a2a3a"/>`;'
'    h+=`<text x="${padL-4}" y="${y+4}" fill="#6b7280" font-size="10" text-anchor="end">${v.toFixed(2)}</text>`;'
'  }'
'  // Bars'
'  LABELS.forEach((lbl,i)=>{'
'    const gx=padL+(i*(groupW+8))+4;'
'    const drawBar=(val,x,col)=>{'
'      if(val===null)return;'
'      const y=yScale(val), bH=H-padB-y;'
'      h+=`<rect x="${x}" y="${y}" width="${cW}" height="${bH}" fill="${col}" rx="2" opacity="0.88"/>`;'
'      h+=`<text x="${x+cW/2}" y="${y-4}" fill="${col}" font-size="9" text-anchor="middle">${val.toFixed(3)}</text>`;'
'    };'
'    drawBar(CON[i],  gx,       "#22c55e");'
'    drawBar(COFF[i], gx+cW+4,  "#f97316");'
'    h+=`<text x="${gx+groupW/2}" y="${H-padB+14}" fill="#9ca3af" font-size="10" text-anchor="middle">${lbl}</text>`;'
'  });'
'  // Legend'
'  h+=`<rect x="${padL}" y="${H-14}" width="12" height="10" fill="#22c55e" rx="2"/>`;'
'  h+=`<text x="${padL+15}" y="${H-5}" fill="#22c55e" font-size="10">WeakRule ON</text>`;'
'  h+=`<rect x="${padL+110}" y="${H-14}" width="12" height="10" fill="#f97316" rx="2"/>`;'
'  h+=`<text x="${padL+125}" y="${H-5}" fill="#f97316" font-size="10">WeakRule OFF</text>`;'
'  svg.setAttribute("height",H);'
'  svg.innerHTML=h;'
'})();'
};

for i=1:numel(js)
    fprintf(fid,'%s\n',js{i});
end
fprintf(fid,'</script>\n</body>\n</html>\n');
fclose(fid);
fprintf('Comparison HTML saved: %s\n', out_path);
end
