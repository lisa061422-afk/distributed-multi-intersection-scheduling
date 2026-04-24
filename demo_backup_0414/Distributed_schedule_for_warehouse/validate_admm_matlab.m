%% validate_admm_matlab.m
% Self-contained MATLAB validation: same scenario as export_config_for_python.m
% Output saved to python_port/matlab_results.txt for Python comparison.
% Run: matlab -batch "run('validate_admm_matlab.m')" from warehouse root.

addpath(genpath('.'));

seed = 201; Nveh = 10;
Dt=2.5; v_max_phys=1.0; detect_range_val=7.6; T_val=0.5; T_ent=0.3; W=1.6;
rho1=1; rho2=1; weight=1.5; max_iter=50;
tol_r=1e-2; tol_s=1e-2; randInitScale=0;

IntSpaceDB = makeIntSpaceDB();

[config_raw,~,~] = generateBalancedTrafficConfig(Nveh, ...
    'Seed', seed, ...
    'MaxPerEntrance',    ceil(Nveh/4), ...
    'MaxPerIntersection', 5, ...
    'MaxPerStage',       ceil(Nveh/4)+1, ...
    'EntrancePenalty',   0.4);

vehicleConfig={}; vid=0;
for g=1:length(config_raw)
    ent=config_raw{g}.entrance; exits=config_raw{g}.exits;
    for j=1:length(exits)
        vid=vid+1;
        vehicleConfig{vid}=struct('entrance',ent,'exits',exits(j),'entryIndex',j);
    end
end
config=vehicleConfig; N=length(config);
pathInfo=getVehiclePaths(config);

pathInfo_agent_chain=cell(1,N); pathInfo_c=cell(1,N);
for n=1:N
    kn=1; int_seq=pathInfo{n}(kn).int; routeIds=pathInfo{n}(kn).routeId;
    dur=zeros(1,numel(int_seq));
    for k_int=1:numel(int_seq)
        ag=int_seq(k_int); rId=routeIds(k_int);
        dur(k_int)=sum(IntSpaceDB{ag}.routeDur{rId});
    end
    ag_chain=[];
    for i=1:length(int_seq)-1
        road_agent=getRoadAgent(int_seq(i),int_seq(i+1));
        ag_chain=[ag_chain,int_seq(i),road_agent];
    end
    ag_chain=[ag_chain,int_seq(end),9];
    pathInfo_agent_chain{n}{kn}=ag_chain; pathInfo_c{n}{kn}=dur;
end

agent_participation=repmat({zeros(N,1)},8,1);
for n=1:N
    kn=1; chain=pathInfo_agent_chain{n}{kn}; ags=chain(1:end-1);
    for ii=1:numel(ags); ag=ags(ii); agent_participation{ag}(n)=1; end
end

v_max=v_max_phys*ones(1,N); detect_range=detect_range_val*ones(1,N);
all_entrances=cellfun(@(c)c.entrance,config);
unique_ents=unique(all_entrances,'stable');
ent_rank=zeros(1,max(unique_ents));
for ei=1:numel(unique_ents); ent_rank(unique_ents(ei))=ei-1; end

alpha_tilde=cell(N,1);
for n=1:N
    alpha_tilde{n}=zeros(1,1);
    base=(detect_range(n)/2-W/2)/v_max(n);
    alpha_tilde{n}(1)=base+ent_rank(config{n}.entrance)*T_ent+(config{n}.entryIndex-1)*T_val;
end

deadline=cell(N,1);
for n=1:N
    kn=1; durations=pathInfo_c{n}{kn}; c_total=sum(durations);
    chain=pathInfo_agent_chain{n}{kn}; num_roads=floor((length(chain)-1)/2);
    deadline{n}=zeros(1,1); deadline{n}(kn)=alpha_tilde{n}(kn)+c_total+Dt*num_roads;
end

% Convert agent_participation from N×1 zero vectors to cell array style
% (run_admm_seq expects cell(N,1) per agent where non-empty means participating)
ap_cell = repmat({cell(N,1)}, 8, 1);
for ag=1:8
    for n=1:N
        if agent_participation{ag}(n)==1
            ap_cell{ag}{n}=1;
        end
    end
end

const=struct();
const.N=N; const.Dt=Dt; const.rho1=rho1; const.rho2=rho2;
const.weight=weight; const.max_iter=max_iter; const.tol_r=tol_r; const.tol_s=tol_s;
const.priority_n=0; const.use_pruning=true; const.use_weak_rule=true;
const.timeout_int_s=30; const.useTBound=true; const.use_quadprog=true;
const.use_adaptive_rho=false; const.randInitScale=randInitScale; const.useParallel=false;
const.alpha_tilde=alpha_tilde; const.deadline=deadline;
const.pathInfo=pathInfo; const.pathInfo_agent_chain=pathInfo_agent_chain;
const.pathInfo_c=pathInfo_c; const.IntSpaceDB=IntSpaceDB; const.config=config;

fprintf('=== MATLAB ADMM Validation ===\n');
fprintf('N=%d  seed=%d  max_iter=%d  Dt=%.1f\n', N, seed, max_iter, Dt);
for n=1:min(3,N)
    fprintf('  Vehicle %d chain: %s\n', n-1, mat2str(pathInfo_agent_chain{n}{1}));
end
fprintf('  alpha_tilde[0..2]: %.4f  %.4f  %.4f\n', ...
    alpha_tilde{1}(1), alpha_tilde{2}(1), alpha_tilde{3}(1));

[x_prev,y_prev,~,res_r,res_s,delay_costs,k_conv,T_admm] = run_admm_seq(const, ap_cell);

fprintf('\n=== Results ===\n');
fprintf('Converged k=%d  elapsed=%.2fs\n', k_conv, T_admm);
fprintf('Delay cost=%.6f\n', delay_costs(k_conv));
fprintf('Final r=%.6f  s=%.6f\n', res_r(k_conv), res_s(k_conv));
fprintf('Residual r history: ');
for i=1:min(k_conv,10); fprintf('%.4f ', res_r(i)); end
fprintf('\n');

fprintf('\nFinal x_prev terminal agent (first 5 vehicles):\n');
for n=1:min(5,N)
    xv=x_prev{9}{n};
    ddl=deadline{n}(1);
    if ~isempty(xv) && ~isnan(xv)
        delay=max(xv-ddl,0);
        fprintf('  n=%d: x=%.4f  deadline=%.4f  delay=%.4f\n', n-1, xv, ddl, delay);
    end
end

% Write to file for comparison
fid=fopen('python_port/matlab_results.txt','w');
fprintf(fid,'Converged k=%d elapsed=%.2f\n', k_conv, T_admm);
fprintf(fid,'Delay cost=%.6f\n', delay_costs(k_conv));
fprintf(fid,'Final r=%.6f s=%.6f\n', res_r(k_conv), res_s(k_conv));
for n=1:min(5,N)
    xv=x_prev{9}{n}; ddl=deadline{n}(1);
    if ~isempty(xv) && ~isnan(xv)
        delay=max(xv-ddl,0);
        fprintf(fid,'n=%d x=%.4f deadline=%.4f delay=%.4f\n', n-1, xv, ddl, delay);
    end
end
fclose(fid);
fprintf('Results written to python_port/matlab_results.txt\n');


%% ── Sequential ADMM core (local function) ──────────────────────────────────
function [x_prev, y_prev, LocalTreeCache, residual_r, residual_s, ...
          delay_costs, k, T_ADMM] = run_admm_seq(const, agent_participation)

N        = const.N;
Dt       = const.Dt;
max_iter = const.max_iter;
tol_r    = const.tol_r;
tol_s    = const.tol_s;
rho1     = const.rho1;

LocalTreeCache = cell(9,1);
[x_prev, y_prev, x_prev_bar, y_prev_bar] = ...
    initEarliestXYPrev_fromChain(const.alpha_tilde, const.pathInfo_agent_chain, ...
                                  const.pathInfo_c, Dt, N, const.randInitScale);

a_x=cell(1,9); a_y=cell(1,9); a_x_new=cell(1,9); a_y_new=cell(1,9);
for i=1:9
    a_x{i}=cell(1,N); a_y{i}=cell(1,N);
    a_x_new{i}=cell(1,N); a_y_new{i}=cell(1,N);
    [a_x{i}{:}]=deal(0); [a_y{i}{:}]=deal(0);
    [a_x_new{i}{:}]=deal(0); [a_y_new{i}{:}]=deal(0);
end

residual_r=zeros(max_iter,1); residual_s=zeros(max_iter,1); delay_costs=zeros(max_iter,1);
T0=tic;

for k=1:max_iter
    fprintf('k=%d\n',k);
    x_last=x_prev; y_last=y_prev;

    %% Step 1: dual variable update
    vehUpd=cell(N,1);
    for n=1:N
        kn=1; chain=const.pathInfo_agent_chain{n}{kn};
        ax_loc=cell(1,9); ay_loc=cell(1,9); xbar_loc=cell(1,9); ybar_loc=cell(1,8);
        for ag=1:9
            ax_loc{ag}=a_x{ag}{n}; ay_loc{ag}=a_y{ag}{n};
            xbar_loc{ag}=x_prev_bar{ag}{n};
            if ag<=8, ybar_loc{ag}=y_prev_bar{ag}{n}; end
        end
        ag0=chain(1);
        xbar_loc{ag0}(kn)=(x_prev{ag0}{n}(kn)+0)/2;
        for pos=2:length(chain)
            prev_ag=chain(pos-1); curr_ag=chain(pos);
            ax_loc{curr_ag}(kn)=a_x{curr_ag}{n}(kn)+rho1*(x_prev{curr_ag}{n}(kn)-y_prev{prev_ag}{n}(kn));
            xbar_loc{curr_ag}(kn)=(x_prev{curr_ag}{n}(kn)+y_prev{prev_ag}{n}(kn))/2;
            ay_loc{prev_ag}(kn)=a_y{prev_ag}{n}(kn)+rho1*(y_prev{prev_ag}{n}(kn)-x_prev{curr_ag}{n}(kn));
            ybar_loc{prev_ag}(kn)=(y_prev{prev_ag}{n}(kn)+x_prev{curr_ag}{n}(kn))/2;
        end
        vehUpd{n}=struct('ax',{ax_loc},'ay',{ay_loc},'xbar',{xbar_loc},'ybar',{ybar_loc});
    end
    for n=1:N
        for ag=1:9
            a_x_new{ag}{n}=vehUpd{n}.ax{ag}; a_y_new{ag}{n}=vehUpd{n}.ay{ag};
            x_prev_bar{ag}{n}=vehUpd{n}.xbar{ag};
            if ag<=8, y_prev_bar{ag}{n}=vehUpd{n}.ybar{ag}; end
        end
    end

    %% Step 2: sequential local updates
    meta=cell(1,9);
    r_local=0;
    for agent_i=1:9
        if agent_i>=1 && agent_i<=4
            entries=agent_participation{agent_i};
            valid_systems=find(~cellfun(@isempty,entries))';
            if isempty(valid_systems)
                meta{agent_i}=struct('kind','intersection','valid_systems',[],'best_x',[],'best_y',[], ...
                    'best_alpha',{{}},'best_gamma',{{}},'cache',[]);
            else
                [best_x,best_y,best_alpha,best_gamma,best_idx,~,cache_out]=...
                    INi_Admm_DecisionTree(agent_i,entries,x_prev,y_prev,...
                        x_prev_bar{agent_i},y_prev_bar{agent_i},...
                        valid_systems,a_x_new{agent_i},a_y_new{agent_i},...
                        const,LocalTreeCache{agent_i});
                if isempty(cache_out)||~isstruct(cache_out), cache_out=struct(); end
                cache_out.iter=k;
                meta{agent_i}=struct('kind','intersection','valid_systems',valid_systems,...
                    'best_x',best_x,'best_y',best_y,...
                    'best_alpha',{best_alpha},'best_gamma',{best_gamma},'cache',cache_out);
            end
        elseif agent_i>=5 && agent_i<=8
            entries=agent_participation{agent_i};
            valid_systems=find(~cellfun(@isempty,entries))';
            if isempty(valid_systems)
                meta{agent_i}=struct('kind','road','valid_systems',[],'x_road',[],'y_road',[]);
            else
                [x_road,y_road]=updateRoadAgent(agent_i,entries,valid_systems,...
                    x_prev{agent_i},y_prev{agent_i},...
                    x_prev_bar{agent_i},y_prev_bar{agent_i},...
                    a_x_new{agent_i},a_y_new{agent_i},const);
                meta{agent_i}=struct('kind','road','valid_systems',valid_systems,...
                    'x_road',x_road,'y_road',y_road);
            end
        else
            [x9_new,delay_cost]=updateAgent9(x_prev{9},x_prev_bar{9},a_x_new{9},const);
            meta{agent_i}=struct('kind','terminal','x9_new',x9_new,'delay_cost',delay_cost);
        end
    end

    %% Merge results
    for agent_i=1:9
        S=meta{agent_i};
        switch S.kind
            case 'intersection'
                if ~isempty(S.valid_systems)
                    LocalTreeCache{agent_i}=S.cache;
                    for n=S.valid_systems
                        kn=1;
                        x_prev{agent_i}{n}(kn)=S.best_x(n);
                        y_prev{agent_i}{n}(kn)=S.best_y(n);
                        r_local=r_local+(S.best_x(n)-S.best_alpha{n}(kn))^2 ...
                                       +(S.best_y(n)-S.best_gamma{n}(kn))^2;
                    end
                end
            case 'road'
                for n=S.valid_systems
                    kn=1;
                    x_prev{agent_i}{n}(kn)=S.x_road(n);
                    y_prev{agent_i}{n}(kn)=S.y_road(n);
                end
            case 'terminal'
                delay_costs(k)=S.delay_cost;
                for n=1:N
                    x_prev{9}{n}(1)=S.x9_new(n);
                end
        end
    end

    %% Step 3: residuals
    r=compute_r(x_prev,y_prev,r_local,const);
    s=0;
    for agent_i=1:8
        ent=agent_participation{agent_i};
        if all(cellfun(@isempty,ent)), continue; end
        vs=find(~cellfun(@isempty,ent))';
        for n=vs
            s=s+norm(x_prev{agent_i}{n}-x_last{agent_i}{n})^2+...
                  norm(y_prev{agent_i}{n}-y_last{agent_i}{n})^2;
        end
    end
    for n=1:N
        s=s+norm(x_prev{9}{n}-x_last{9}{n})^2;
    end

    residual_r(k)=r; residual_s(k)=s;
    a_x=a_x_new; a_y=a_y_new;

    fprintf('  r=%.6f  s=%.6f\n',r,s);
    if r<tol_r && s<tol_s
        fprintf('Converged at k=%d\n',k);
        residual_r=residual_r(1:k); residual_s=residual_s(1:k);
        delay_costs=delay_costs(1:k);
        break;
    end
end

T_ADMM=toc(T0);  % seconds
end
