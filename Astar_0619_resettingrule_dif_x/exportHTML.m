function exportHTML(NODES, LEAF, cfg, out_path, NODES2, LEAF2)
% exportHTML  Auto-generates a self-contained interactive HTML demo after Main.m.
%
%   exportHTML(NODES, LEAF, cfg)
%   exportHTML(NODES, LEAF, cfg, 'my_demo.html')
%   exportHTML(NODES_ON, LEAF_ON, cfg, [], NODES_OFF, LEAF_OFF)
%       — generates a single HTML with ON/OFF toggle button at the top.
%
%   NODES/LEAF  : WeakRule ON results  (cfg.useWeakRule should be true)
%   NODES2/LEAF2: WeakRule OFF results (optional)

if nargin < 4 || isempty(out_path)
    out_path = fullfile(fileparts(mfilename('fullpath')), 'astar_demo.html');
end
has_off = nargin >= 6 && ~isempty(NODES2) && ~isempty(LEAF2);

N = cfg.N;  M = cfg.M;  C = cfg.C;  Ni = cfg.Ni;  Map = cfg.Map;  S = cfg.S;
useWeak = true;   % first dataset is always ON when called from Main.m

%% Build dataset for one NODES/LEAF pair
[tn, te, ls, nL] = buildDataset(NODES, LEAF, cfg);

%% Build second dataset (WeakRule OFF) if provided
if has_off
    cfg2        = cfg;
    cfg2.useWeakRule = false;
    [tn2, te2, ls2, nL2] = buildDataset(NODES2, LEAF2, cfg2);
else
    tn2 = []; te2 = []; ls2 = []; nL2 = 0;
end

%% Write HTML file
fid = fopen(out_path, 'w', 'n', 'UTF-8');
if fid < 0,  error('exportHTML: cannot write to %s', out_path);  end

writeHead(fid, N, M, nL, nL2, has_off);
writeDataScript(fid, tn, te, ls, nL, tn2, te2, ls2, nL2, cfg, has_off);
writeRenderScript(fid);

fprintf(fid, '</body>\n</html>\n');
fclose(fid);
fprintf('HTML demo saved: %s  (ON:%d leaves  OFF:%d leaves)\n', out_path, nL, nL2);
end


% =========================================================================
function [tn_out, te_out, ls_out, nL] = buildDataset(NODES, LEAF, cfg)
% Compress tree + pre-compute all leaf schedules for one WeakRule setting.
if isempty(LEAF)
    tn_out = []; te_out = []; ls_out = {}; nL = 0;
    return;
end
gcosts = arrayfun(@(l) NODES{l}{10}, LEAF);
[~, sidx] = sort(gcosts, 'ascend');
LEAF_s    = LEAF(sidx);
nL        = numel(LEAF_s);
num_nodes = size(NODES, 1);

children = cell(num_nodes, 1);
for i = 1:num_nodes
    p = NODES{i}{7};
    if p > 0 && p <= num_nodes,  children{p}(end+1) = i;  end
end
is_branch = false(num_nodes,1);  is_lf = false(num_nodes,1);
for i = 1:num_nodes
    nc = numel(children{i});
    if nc >= 2, is_branch(i) = true; end
    if nc == 0, is_lf(i)     = true; end
end
is_disp = false(num_nodes,1);  is_disp(1) = true;
for i = 1:num_nodes
    if is_branch(i), is_disp(i) = true; is_disp(children{i}) = true; end
    if is_lf(i),     is_disp(i) = true; end
end
disp_nodes = find(is_disp);  nd = numel(disp_nodes);
old2new = zeros(num_nodes,1);
for k = 1:nd,  old2new(disp_nodes(k)) = k;  end

src_list = [];  dst_list = [];
for k = 1:nd
    orig = disp_nodes(k);  if orig == 1, continue; end
    p = NODES{orig}{7};
    while p > 0 && ~is_disp(p),  p = NODES{p}{7};  end
    if p > 0 && is_disp(p)
        src_list(end+1) = old2new(p)    - 1; %#ok<AGROW>
        dst_list(end+1) = old2new(orig) - 1; %#ok<AGROW>
    end
end
leaf_comp0 = old2new(LEAF_s) - 1;

ls_out = cell(1, nL);
gcosts_s = arrayfun(@(l) NODES{l}{10}, LEAF_s);
for li = 1:nL
    ls_out{li} = computeLeafSched(NODES, LEAF_s(li), cfg);
    ls_out{li}.compId = leaf_comp0(li);
    ls_out{li}.gcost  = gcosts_s(li);
end

% Pack tree nodes: [tw, g, leafRank]
tn_out = zeros(nd, 3);
for k = 1:nd
    orig = disp_nodes(k);
    lr = find(leaf_comp0 == k-1, 1);
    if isempty(lr), lr = 0; else, lr = lr; end   % 1-based rank
    tn_out(k,:) = [NODES{orig}{5}, NODES{orig}{10}, lr-1];  % lr-1: 0-based, -1 if not leaf
end

te_out = [src_list(:), dst_list(:)];
end


% =========================================================================
function s = computeLeafSched(NODES, leaf_idx, cfg)
N = cfg.N;  C = cfg.C;  Ni = cfg.Ni;  Map = cfg.Map;  S = cfg.S;

gamma_last = NODES{leaf_idx}{11};

alpha_arr = cell(1, N);
for n = 1:N
    alpha_arr{n} = nan(1, Ni(n));
    for i = 1:Ni(n)
        if ~isempty(gamma_last{n}) && numel(gamma_last{n}) >= i && gamma_last{n}(i) > 0
            alpha_arr{n}(i) = gamma_last{n}(i) - sum(C(:, n));
        end
    end
end

% Space blocks: rows = [ts, te, m_0based, n_0based]
spBlks = zeros(0, 4);
for n = 1:N
    for i = 1:Ni(n)
        if isnan(alpha_arr{n}(i)), continue; end
        alpha_n = alpha_arr{n}(i);
        t_acc = 0;
        for ss = 1:S
            m = Map(ss, n);
            if m > 0 && C(ss, n) > 1e-9
                spBlks(end+1, :) = [alpha_n+t_acc, alpha_n+t_acc+C(ss,n), m-1, n-1]; %#ok<AGROW>
            end
            t_acc = t_acc + C(ss, n);
        end
    end
end

% Vehicle timelines: cell{n}(i,:) = [alpha_n, gamma_n]
vehicles = cell(1, N);
for n = 1:N
    vt = nan(Ni(n), 2);
    for i = 1:Ni(n)
        if ~isnan(alpha_arr{n}(i))
            vt(i, 1) = alpha_arr{n}(i);
            vt(i, 2) = gamma_last{n}(i);
        end
    end
    vehicles{n} = vt;
end

s.gcost    = NODES{leaf_idx}{10};
s.compId   = -1;
s.spBlks   = spBlks;
s.vehicles = vehicles;
end


% =========================================================================
function writeHead(fid, N, M, nL_on, nL_off, has_off)

fprintf(fid, '<!DOCTYPE html>\n<html lang="en">\n<head>\n');
fprintf(fid, '<meta charset="UTF-8">\n');
fprintf(fid, '<title>A* Scheduler — N=%d M=%d</title>\n', N, M);
fprintf(fid, '<style>\n');
fprintf(fid, ':root{--bg:#08111b;--panel:#0c1826;--border:rgba(255,255,255,.1);--text:#eef4ff;--muted:#b6c3d8;}\n');
fprintf(fid, '*{box-sizing:border-box;margin:0;padding:0}\n');
fprintf(fid, 'html,body{width:100%%;height:100%%;overflow:hidden;');
fprintf(fid, 'background:radial-gradient(circle at top,#102133 0%%,#08111b 42%%,#04090f 100%%);');
fprintf(fid, 'color:var(--text);font-family:Inter,system-ui,sans-serif;font-size:12px}\n');
fprintf(fid, '#app{display:flex;flex-direction:column;height:100vh}\n');
fprintf(fid, '#topbar{display:flex;align-items:center;gap:10px;padding:5px 10px;');
fprintf(fid, 'background:#060e18;border-bottom:1px solid var(--border);flex-shrink:0}\n');
fprintf(fid, '#topbar b{color:#79caff;font-size:13px}\n');
fprintf(fid, '#main{display:flex;flex:1;overflow:hidden}\n');
fprintf(fid, '#left{width:38vw;min-width:240px;display:flex;flex-direction:column;');
fprintf(fid, 'border-right:1px solid var(--border);padding:6px;gap:5px}\n');
fprintf(fid, '#tree-wrap{flex:1;background:#060e18;border:1px solid var(--border);');
fprintf(fid, 'border-radius:6px;overflow:hidden;position:relative;cursor:grab}\n');
fprintf(fid, '#tree-wrap.drag{cursor:grabbing}\n');
fprintf(fid, '#tsv{position:absolute;top:0;left:0;transform-origin:0 0}\n');
fprintf(fid, '#right{flex:1;overflow-y:auto;padding:8px 10px}\n');
fprintf(fid, '#ssv{display:block}\n');
fprintf(fid, 'circle.node{cursor:pointer;transition:r .1s,fill .1s}\n');
fprintf(fid, 'circle.node:hover{r:8!important;filter:brightness(1.3)}\n');
% Toggle button styles
fprintf(fid, '.rule-btn{padding:5px 16px;border-radius:20px;border:2px solid;');
fprintf(fid, 'font-size:12px;font-weight:700;cursor:pointer;transition:all .2s;background:transparent}\n');
fprintf(fid, '.rule-btn.on{border-color:#22c55e;color:#22c55e}\n');
fprintf(fid, '.rule-btn.on.active{background:#22c55e;color:#000}\n');
fprintf(fid, '.rule-btn.off{border-color:#f97316;color:#f97316}\n');
fprintf(fid, '.rule-btn.off.active{background:#f97316;color:#000}\n');
fprintf(fid, '#sel-info{font-size:11px;color:#9ca3af;margin-left:6px}\n');
fprintf(fid, '</style>\n</head>\n<body>\n');

fprintf(fid, '<div id="app">\n');
fprintf(fid, '  <div id="topbar">\n');
fprintf(fid, '    <b>A* Intersection Scheduler</b>\n');
fprintf(fid, '    <span style="color:#6b7280">N=%d&nbsp;M=%d</span>\n', N, M);
if has_off
    fprintf(fid, '    <button class="rule-btn on active" onclick="switchRule(true)">WeakRule ON&nbsp;(%d leaves)</button>\n', nL_on);
    fprintf(fid, '    <button class="rule-btn off" onclick="switchRule(false)">WeakRule OFF&nbsp;(%d leaves)</button>\n', nL_off);
else
    fprintf(fid, '    <button class="rule-btn on active">WeakRule ON&nbsp;(%d leaves)</button>\n', nL_on);
end
fprintf(fid, '    <span id="sel-info">click a leaf &nbsp;|&nbsp; &larr;&rarr; to cycle</span>\n');
fprintf(fid, '  </div>\n');
fprintf(fid, '  <div id="main">\n');
fprintf(fid, '    <div id="left"><div id="tree-wrap"><svg id="tsv"></svg></div></div>\n');
fprintf(fid, '    <div id="right"><svg id="ssv"></svg></div>\n');
fprintf(fid, '  </div>\n');
fprintf(fid, '</div>\n');
end


% =========================================================================
function writeDataScript(fid, tn, te, ls, nL, tn2, te2, ls2, nL2, cfg, has_off)
% Writes both datasets as D = [dataset_ON, dataset_OFF].
% JS accesses current dataset via D[activeRule].TN / .TE / .LS

N = cfg.N;  M = cfg.M;

fprintf(fid, '<script>\n');
fprintf(fid, '// Auto-generated by exportHTML.m\n');

% CFG (alpha_tilde is the same for both rules — only initial conditions matter)
fprintf(fid, 'const CFG={N:%d,M:%d,alphaTilde:[', N, M);
% (reuse CFG from the ON run — alpha_tilde doesn't depend on WeakRule)
for n = 1:N
    if n > 1, fprintf(fid, ','); end
    fprintf(fid, '[');
    at_n = cfg.alpha_tilde{n};
    for i = 1:numel(at_n)
        if i > 1, fprintf(fid, ','); end
        fprintf(fid, '%.4f', at_n(i));
    end
    fprintf(fid, ']');
end
fprintf(fid, ']};\n');

% Write both datasets as D[0]=ON, D[1]=OFF
fprintf(fid, 'const D=[');
writeOneDataset(fid, tn, te, ls, nL, N);
if has_off && ~isempty(tn2)
    fprintf(fid, ',');
    writeOneDataset(fid, tn2, te2, ls2, nL2, N);
end
fprintf(fid, '];\n');
fprintf(fid, 'const HAS_OFF=%s;\n', lower(mat2str(has_off)));
fprintf(fid, '</script>\n');
end


% =========================================================================
function writeOneDataset(fid, tn, te, ls, nL, N)
% Write one {TN, TE, LS} object into the JS array.
fprintf(fid, '{\n');

% TN: tree nodes [tw, g, leafRank]
fprintf(fid, 'TN:[\n');
for k = 1:size(tn,1)
    if k > 1, fprintf(fid, ','); end
    fprintf(fid, '[%.3f,%.4f,%d]\n', tn(k,1), tn(k,2), tn(k,3));
end
fprintf(fid, '],\n');

% TE: edges
fprintf(fid, 'TE:[\n');
for e = 1:size(te,1)
    if e > 1, fprintf(fid, ','); end
    fprintf(fid, '[%d,%d]\n', te(e,1), te(e,2));
end
fprintf(fid, '],\n');

% LS: leaf schedules
fprintf(fid, 'LS:[\n');
for li = 1:nL
    s = ls{li};
    if li > 1, fprintf(fid, ','); end
    fprintf(fid, '{g:%.4f,c:%d,b:[', s.gcost, s.compId);
    for bi = 1:size(s.spBlks,1)
        if bi > 1, fprintf(fid, ','); end
        fprintf(fid, '[%.4f,%.4f,%d,%d]', ...
            s.spBlks(bi,1), s.spBlks(bi,2), s.spBlks(bi,3), s.spBlks(bi,4));
    end
    fprintf(fid, '],v:[');
    for n = 1:N
        if n > 1, fprintf(fid, ','); end
        vt = s.vehicles{n};
        fprintf(fid, '[');
        for i = 1:size(vt,1)
            if i > 1, fprintf(fid, ','); end
            if isnan(vt(i,1)), fprintf(fid, 'null');
            else, fprintf(fid, '[%.4f,%.4f]', vt(i,1), vt(i,2)); end
        end
        fprintf(fid, ']');
    end
    fprintf(fid, ']}\n');
end
fprintf(fid, ']\n');
fprintf(fid, '}');
end


% =========================================================================
function writeRenderScript(fid)

js = {
'<script>'
'// ===== Space-indexed colors (matching MATLAB plotInteractiveTree) ======='
'const MCOLS = ["#00ff00","#ffd900","#ff8000","#00b3e6","#ff0000","#ffffff"];'
'function mCol(m){ return m>=0 && m<MCOLS.length ? MCOLS[m] : "#aaaaaa"; }'
''
'// ===== State ==========================================================='
'let activeRule = 0;   // 0 = WeakRule ON (D[0]),  1 = WeakRule OFF (D[1])'
'let selRank    = 0;'
'let treeCache  = {};  // layout cache per rule index'
''
'// ===== Per-dataset tree layout ========================================='
'function getLayout(ri){'
'  if(treeCache[ri]) return treeCache[ri];'
'  const {TN,TE}=D[ri];'
'  const n=TN.length;'
'  const chld=Array.from({length:n},()=>[]);'
'  const par=new Int32Array(n).fill(-1);'
'  TE.forEach(([s,d])=>{chld[s].push(d); par[d]=s;});'
'  const dep=new Int32Array(n).fill(-1); dep[0]=0;'
'  const q=[0];'
'  for(let i=0;i<q.length;i++) chld[q[i]].forEach(v=>{if(dep[v]<0){dep[v]=dep[q[i]]+1;q.push(v);}});'
'  const maxDep=Math.max(...dep);'
'  const xPos=new Float32Array(n);'
'  let lc=0;'
'  function lay(u){if(!chld[u].length){xPos[u]=lc++;return;} chld[u].forEach(lay); xPos[u]=(xPos[chld[u][0]]+xPos[chld[u][chld[u].length-1]])/2;}'
'  lay(0);'
'  treeCache[ri]={chld,par,dep,maxDep,xPos,leafCnt:lc};'
'  return treeCache[ri];'
'}'
''
'// ===== Switch WeakRule ================================================='
'function switchRule(useWeak){'
'  activeRule = useWeak ? 0 : 1;'
'  if(activeRule >= D.length) return;'
'  selRank = 0;'
'  // Update button styles'
'  document.querySelectorAll(".rule-btn").forEach(b=>{'
'    b.classList.toggle("active", b.classList.contains(useWeak?"on":"off"));'
'  });'
'  renderTree();'
'  renderSched(0);'
'}'
''
'// ===== Tree rendering =================================================='
'function renderTree(){'
'  const {TN,TE,LS}=D[activeRule];'
'  const {chld,par,dep,maxDep,xPos,leafCnt}=getLayout(activeRule);'
'  const wrap=document.getElementById("tree-wrap");'
'  const W=Math.max(wrap.clientWidth, leafCnt*14+20);'
'  const H=Math.max(wrap.clientHeight, (maxDep+1)*42+20);'
'  const svg=document.getElementById("tsv");'
'  svg.setAttribute("width",W); svg.setAttribute("height",H);'
''
'  const xPad=10,yPad=16;'
'  const xSp=(W-2*xPad)/Math.max(leafCnt-1,1);'
'  const yStep=(H-2*yPad)/Math.max(maxDep,1);'
'  const px=i=>xPad+xPos[i]*xSp;'
'  const py=i=>yPad+dep[i]*yStep;'
''
'  const pathSet=new Set();'
'  if(selRank>=0 && selRank<LS.length){'
'    let u=LS[selRank].c;'
'    while(u>=0){pathSet.add(u); u=par[u];}'
'  }'
''
'  let h="";'
'  TE.forEach(([s,d])=>{'
'    const hot=pathSet.has(s)&&pathSet.has(d);'
'    h+=`<line x1="${px(s)}" y1="${py(s)}" x2="${px(d)}" y2="${py(d)}" `+'
'      `stroke="${hot?"#ef4444":"#1e3a5f"}" stroke-width="${hot?2.5:1}"/>`;'
'  });'
'  TN.forEach(([tw,g,lr],i)=>{'
'    const isLeaf=lr>=0, isBest=lr===0, isSel=pathSet.has(i);'
'    let fill=isSel?"#ef4444":isBest?"#22c55e":isLeaf?"#f97316":"#3b82f6";'
'    const r=isLeaf?6:4;'
'    h+=`<circle class="node" cx="${px(i)}" cy="${py(i)}" r="${r}" fill="${fill}" `+'
'      `data-lr="${lr}"/>`;'
'    if(isLeaf&&lr<40){'
'      h+=`<text x="${px(i)+7}" y="${py(i)-4}" fill="#8899aa" `+'
'        `font-size="9" font-family="monospace">#${lr+1} g=${g.toFixed(2)}</text>`;'
'    }'
'  });'
'  svg.innerHTML=h;'
'  svg.querySelectorAll("circle.node").forEach(el=>{'
'    el.addEventListener("click",e=>{'
'      e.stopPropagation();'
'      const lr=parseInt(el.dataset.lr);'
'      if(lr>=0) selectLeaf(lr);'
'    });'
'  });'
'}'
''
'// ===== Schedule rendering (matches MATLAB plotInteractiveTree) =========='
'function renderSched(rank){'
'  const {LS}=D[activeRule];'
'  const sv=document.getElementById("ssv");'
'  if(rank<0||rank>=LS.length){sv.innerHTML="";return;}'
'  const sch=LS[rank];'
'  const W=(sv.parentElement.clientWidth||700)-10;'
'  const nRows=CFG.N+CFG.M;'
'  const rowH=42, labelW=44, padTop=26, axH=26;'
'  const H=padTop+nRows*rowH+axH;'
'  sv.setAttribute("width",W);'
'  sv.setAttribute("height",H);'
''
'  // Time range: include alpha_tilde (generation times) so approach phase is visible'
'  let tMin=Infinity,tMax=-Infinity;'
'  CFG.alphaTilde.forEach(trips=>trips.forEach(at=>{if(at<tMin)tMin=at;}));'
'  sch.b.forEach(([ts,te])=>{if(ts<tMin)tMin=ts;if(te>tMax)tMax=te;});'
'  if(!isFinite(tMin)){tMin=0;tMax=20;}'
'  const span=Math.max(tMax-tMin,1);'
'  const pad=0.05*span;'
'  tMin-=pad; tMax+=pad;'
'  const tSc=t=>labelW+(t-tMin)/(tMax-tMin)*(W-labelW-6);'
''
'  let h=`<rect width="${W}" height="${H}" fill="#0a1520"/>`;'
''
'  // Title'
'  const opt=rank===0;'
'  const tclr=opt?"#22c55e":"#93c5fd";'
'  const ttl=opt'
'    ?`Path ${rank+1}  \u2502  g\u202f=\u202f${sch.g.toFixed(4)}  \u2605 OPTIMAL`'
'    :`Path ${rank+1}  \u2502  g\u202f=\u202f${sch.g.toFixed(4)}    (optimal: ${LS[0].g.toFixed(4)})`;'
'  h+=`<text x="${W/2}" y="17" fill="${tclr}" font-size="13" font-weight="bold" `+'
'    `text-anchor="middle" font-family="Inter,system-ui,sans-serif">${ttl}</text>`;'
''
'  // Grid lines (vertical, light)'
'  const nTk=8;'
'  for(let t=0;t<=nTk;t++){'
'    const tv=tMin+t*(tMax-tMin)/nTk;'
'    const tx=tSc(tv);'
'    h+=`<line x1="${tx}" y1="${padTop}" x2="${tx}" y2="${padTop+nRows*rowH}" `+'
'      `stroke="#1a2a3a" stroke-width="1"/>`;'
'  }'
''
'  // Row backgrounds + labels'
'  for(let r=0;r<nRows;r++){'
'    const ry=padTop+r*rowH;'
'    const isSpc=r>=CFG.N;'
'    h+=`<rect x="${labelW}" y="${ry+1}" width="${W-labelW-6}" height="${rowH-2}" `+'
'      `fill="${isSpc?"#0a1018":"#0d1520"}" rx="2"/>`;'
'    const lbl=r<CFG.N?`N${r+1}`:`M${r-CFG.N+1}`;'
'    h+=`<text x="${labelW-5}" y="${ry+rowH/2+4}" fill="#8899aa" `+'
'      `font-size="11" text-anchor="end" font-family="Inter,system-ui,sans-serif">${lbl}</text>`;'
'  }'
''
'  // ── Vehicle rows (N rows) ─────────────────────────────────────────────'
'  for(let n=0;n<CFG.N;n++){'
'    const ry=padTop+n*rowH;'
'    const trips=sch.v[n];'
'    const atRef=CFG.alphaTilde[n];'
''
'    trips.forEach((ag,i)=>{'
'      if(!ag) return;'
'      const [al,gm]=ag;'
'      const xAT=tSc(atRef[i]), xAl=tSc(al);'
''
'      // Light-blue approach bar at half height (detection zone → merging zone)'
'      if(xAl>xAT){'
'        const midY=ry+rowH*0.5-1;'
'        h+=`<rect x="${xAT}" y="${ry+rowH*0.35}" width="${xAl-xAT}" `+'
'          `height="${rowH*0.3}" fill="#87ceeb" opacity="0.25" rx="1"/>`;'
'        h+=`<line x1="${xAT}" y1="${midY}" x2="${xAl}" y2="${midY}" `+'
'          `stroke="#87ceeb" stroke-width="2"/>`;'
'      }'
''
'      // Blue dashed vertical line at alpha_tilde (enters detection zone)'
'      h+=`<line x1="${xAT}" y1="${ry+2}" x2="${xAT}" y2="${ry+rowH-2}" `+'
'        `stroke="#4488ff" stroke-width="1.5" stroke-dasharray="4,3"/>`;'
''
'      // Red dashed vertical line at alpha_n (enters merging zone)'
'      h+=`<line x1="${xAl}" y1="${ry+2}" x2="${xAl}" y2="${ry+rowH-2}" `+'
'        `stroke="#ff4444" stroke-width="1.5" stroke-dasharray="4,3"/>`;'
'    });'
''
'    // Space occupation: full-height colored fills, color = space color'
'    sch.b.filter(b=>b[3]===n).sort((a,b2)=>a[0]-b2[0]).forEach(([ts,te,m])=>{'
'      const x1=tSc(ts),x2=tSc(te);'
'      const col=mCol(m);'
'      h+=`<rect x="${x1}" y="${ry+2}" width="${Math.max(x2-x1,1)}" `+'
'        `height="${rowH-4}" fill="${col}" opacity="0.92" rx="1"/>`;'
'    });'
'  }'
''
'  // ── Space rows (M rows) ───────────────────────────────────────────────'
'  for(let m=0;m<CFG.M;m++){'
'    const ry=padTop+(CFG.N+m)*rowH;'
'    const col=mCol(m);'
'    sch.b.filter(b=>b[2]===m).sort((a,b2)=>a[0]-b2[0]).forEach(([ts,te,,n])=>{'
'      const x1=tSc(ts),x2=tSc(te);'
'      h+=`<rect x="${x1}" y="${ry+4}" width="${Math.max(x2-x1,1)}" `+'
'        `height="${rowH-8}" fill="white" stroke="${col}" stroke-width="2.5" rx="2"/>`;'
'      if(x2-x1>20){'
'        h+=`<text x="${(x1+x2)/2}" y="${ry+rowH/2+4}" fill="${col}" `+'
'          `font-size="11" font-weight="bold" text-anchor="middle" `+'
'          `font-family="Inter,system-ui,sans-serif">N${n+1}</text>`;'
'      }'
'    });'
'  }'
''
'  // Time axis'
'  const axY=padTop+nRows*rowH;'
'  h+=`<line x1="${labelW}" y1="${axY}" x2="${W-6}" y2="${axY}" stroke="#334155"/>`;'
'  for(let t=0;t<=nTk;t++){'
'    const tv=tMin+t*(tMax-tMin)/nTk;'
'    const tx=tSc(tv);'
'    h+=`<line x1="${tx}" y1="${axY}" x2="${tx}" y2="${axY+5}" stroke="#334155"/>`;'
'    h+=`<text x="${tx}" y="${axY+15}" fill="#6b7280" font-size="10" `+'
'      `text-anchor="middle" font-family="monospace">${tv.toFixed(1)}</text>`;'
'  }'
'  h+=`<text x="${(labelW+W-6)/2}" y="${H-2}" fill="#6b7280" `+'
'    `font-size="10" text-anchor="middle">Time (s)</text>`;'
''
'  sv.innerHTML=h;'
'}'
''
'// ===== Interaction ====================================================='
'function selectLeaf(rank){'
'  const {LS}=D[activeRule];'
'  selRank=rank;'
'  renderTree();'
'  renderSched(rank);'
'  const ruleLabel=activeRule===0?"<span style=color:#22c55e>WeakRule ON</span>":"<span style=color:#f97316>WeakRule OFF</span>";'
'  const g=LS[rank].g;'
'  const info=rank===0'
'    ?`${ruleLabel} &nbsp;|&nbsp; <b style="color:#22c55e">Path ${rank+1} — OPTIMAL &nbsp; g=${g.toFixed(4)}</b>`'
'    :`${ruleLabel} &nbsp;|&nbsp; Path ${rank+1}/${LS.length} &nbsp; g=${g.toFixed(4)} &nbsp;(optimal: ${LS[0].g.toFixed(4)})`;'
'  document.getElementById("sel-info").innerHTML=info;'
'}'
''
'document.addEventListener("keydown",e=>{'
'  const {LS}=D[activeRule];'
'  if(e.key==="ArrowRight") selectLeaf(Math.min(selRank+1,LS.length-1));'
'  if(e.key==="ArrowLeft")  selectLeaf(Math.max(selRank-1,0));'
'});'
''
'// ===== Pan/zoom on tree ================================================'
'(function(){'
'  const wrap=document.getElementById("tree-wrap");'
'  const svg=document.getElementById("tsv");'
'  let ox=0,oy=0,sc=1,drag=false,sx=0,sy=0,sox=0,soy=0;'
'  function applyT(){svg.style.transform=`translate(${ox}px,${oy}px) scale(${sc})`;}'
'  wrap.addEventListener("wheel",e=>{'
'    e.preventDefault();'
'    const f=e.deltaY<0?1.12:0.9;'
'    const r=wrap.getBoundingClientRect();'
'    const mx=e.clientX-r.left,my=e.clientY-r.top;'
'    ox=mx-(mx-ox)*f; oy=my-(my-oy)*f; sc*=f; applyT();'
'  },{passive:false});'
'  wrap.addEventListener("mousedown",e=>{'
'    drag=true;sx=e.clientX;sy=e.clientY;sox=ox;soy=oy;wrap.classList.add("drag");'
'  });'
'  document.addEventListener("mouseup",()=>{drag=false;wrap.classList.remove("drag");});'
'  document.addEventListener("mousemove",e=>{'
'    if(!drag)return; ox=sox+(e.clientX-sx);oy=soy+(e.clientY-sy);applyT();'
'  });'
'})();'
''
'// ===== Init ============================================================'
'window.addEventListener("load",()=>{'
'  if(!D || !D.length){document.getElementById("sel-info").textContent="No data";return;}'
'  renderTree(); selectLeaf(0);'
'});'
'window.addEventListener("resize",()=>{renderTree();renderSched(selRank);});'
'</script>'
};

for i = 1:numel(js)
    fprintf(fid, '%s\n', js{i});
end
end
