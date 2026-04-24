%% export_config_for_python.m
% Exports one scenario's full configuration to JSON so Python can
% run the same ADMM and produce numerically comparable results.
%
% Usage: set seed and Nveh below, then run from the warehouse folder.
% Output: python_port/validation_config.json

seed = 201;
Nveh = 10;

%% ---- Shared physical parameters (copy from MAIN_Parallel_Compute) ----
Dt              = 2.5;
v_max_phys      = 1.0;
detect_range_val= 7.6;
T_val           = 0.5;
T_ent           = 0.3;
W               = 1.6;

rho1 = 1; rho2 = 1; weight = 1.5; max_iter = 50;   % short run for validation
tol_r = 1e-2; tol_s = 1e-2;
randInitScale = 0;   % deterministic init for reproducible comparison

%% ---- IntSpaceDB ----
IntSpaceDB = makeIntSpaceDB();

%% ---- Config generation ----
[config_raw, ~, ~] = generateBalancedTrafficConfig(Nveh, ...
    'Seed', seed, ...
    'MaxPerEntrance',    ceil(Nveh/4), ...
    'MaxPerIntersection', 5, ...
    'MaxPerStage',       ceil(Nveh/4) + 1, ...
    'EntrancePenalty',   0.4);

vehicleConfig = {};
vid = 0;
for g = 1:length(config_raw)
    ent   = config_raw{g}.entrance;
    exits = config_raw{g}.exits;
    for j = 1:length(exits)
        vid = vid + 1;
        vehicleConfig{vid} = struct('entrance', ent, 'exits', exits(j), 'entryIndex', j);
    end
end
config = vehicleConfig;
N = length(config);

%% ---- Routing ----
pathInfo = getVehiclePaths(config);

pathInfo_agent_chain = cell(1, N);
pathInfo_c           = cell(1, N);
for n = 1:N
    kn = 1;
    pathInfo_agent_chain{n} = cell(1, 1);
    pathInfo_c{n}           = cell(1, 1);
    int_seq  = pathInfo{n}(kn).int;
    routeIds = pathInfo{n}(kn).routeId;

    dur = zeros(1, numel(int_seq));
    for k_int = 1:numel(int_seq)
        ag  = int_seq(k_int);
        rId = routeIds(k_int);
        dur(k_int) = sum(IntSpaceDB{ag}.routeDur{rId});
    end

    ag_chain = [];
    for i = 1:length(int_seq)-1
        road_agent = getRoadAgent(int_seq(i), int_seq(i+1));
        ag_chain   = [ag_chain, int_seq(i), road_agent];
    end
    ag_chain = [ag_chain, int_seq(end), 9];
    pathInfo_agent_chain{n}{kn} = ag_chain;
    pathInfo_c{n}{kn}           = dur;
end

%% ---- agent_participation ----
agent_participation = repmat({zeros(N, 1)}, 8, 1);
for n = 1:N
    kn    = 1;
    chain = pathInfo_agent_chain{n}{kn};
    ags   = chain(1:end-1);
    for ii = 1:numel(ags)
        ag = ags(ii);
        agent_participation{ag}(n) = 1;
    end
end

%% ---- alpha_tilde, deadline, initial_position ----
v_max        = v_max_phys * ones(1, N);
detect_range = detect_range_val * ones(1, N);
headway      = T_val;

all_entrances = cellfun(@(c) c.entrance, config);
unique_ents   = unique(all_entrances, 'stable');
ent_rank      = zeros(1, max(unique_ents));
for ei = 1:numel(unique_ents)
    ent_rank(unique_ents(ei)) = ei - 1;
end

alpha_tilde      = cell(N, 1);
initial_position = zeros(N, 1);
for n = 1:N
    alpha_tilde{n} = zeros(1, 1);
    base = (detect_range(n)/2 - W/2) / v_max(n);
    alpha_tilde{n}(1) = base ...
        + ent_rank(config{n}.entrance) * T_ent ...
        + (config{n}.entryIndex - 1)   * headway;
    initial_position(n) = -(detect_range(n)/2 - W/2 + alpha_tilde{n}(1) * v_max(n));
end

deadline = cell(N, 1);
for n = 1:N
    kn       = 1;
    durations = pathInfo_c{n}{kn};
    c_total   = sum(durations);
    chain     = pathInfo_agent_chain{n}{kn};
    num_roads = floor((length(chain) - 1) / 2);
    deadline{n} = zeros(1, 1);
    deadline{n}(kn) = alpha_tilde{n}(kn) + c_total + Dt * num_roads;
end

%% ---- Serialise to JSON ----
out = struct();
out.N   = N;
out.Dt  = Dt;
out.rho1 = rho1;  out.rho2 = rho2;
out.weight = weight;
out.max_iter = max_iter;
out.tol_r = tol_r;  out.tol_s = tol_s;
out.randInitScale = randInitScale;
out.seed = seed;

% alpha_tilde and deadline: N-element arrays (kn=1 only)
out.alpha_tilde = zeros(N, 1);
out.deadline    = zeros(N, 1);
for n = 1:N
    out.alpha_tilde(n) = alpha_tilde{n}(1);
    out.deadline(n)    = deadline{n}(1);
end
out.initial_position = initial_position;

% pathInfo_agent_chain: N x 1 cell → N-element list of integer arrays (1-indexed agents)
chains_cell = cell(N, 1);
for n = 1:N
    chains_cell{n} = pathInfo_agent_chain{n}{1};
end
out.agent_chains = chains_cell;

% pathInfo_c: N-element list of duration arrays
cseqs_cell = cell(N, 1);
for n = 1:N
    cseqs_cell{n} = pathInfo_c{n}{1};
end
out.path_c = cseqs_cell;

% pathInfo: per-vehicle routeId arrays (1-indexed route labels)
route_ids_cell = cell(N, 1);
for n = 1:N
    route_ids_cell{n} = pathInfo{n}(1).routeId;
end
out.route_ids = route_ids_cell;

% agent_participation: 8 x N binary matrix
ap_mat = zeros(8, N);
for ag = 1:8
    ap_mat(ag, :) = agent_participation{ag}(:)';
end
out.agent_participation = ap_mat;

% IntSpaceDB: 4 intersections
int_db_out = cell(4, 1);
for ag = 1:4
    db = struct();
    db.numSpaces  = IntSpaceDB{ag}.numSpaces;
    db.numRoutes  = IntSpaceDB{ag}.numRoutes;
    rs_cell = cell(1, db.numRoutes);
    rd_cell = cell(1, db.numRoutes);
    for r = 1:db.numRoutes
        rs_cell{r} = IntSpaceDB{ag}.routeSpace{r};
        rd_cell{r} = IntSpaceDB{ag}.routeDur{r};
    end
    db.routeSpace = rs_cell;
    db.routeDur   = rd_cell;
    int_db_out{ag} = db;
end
out.IntSpaceDB = int_db_out;

% Write JSON
outFile = fullfile('python_port', 'validation_config.json');
fid = fopen(outFile, 'w');
fprintf(fid, '%s', jsonencode(out));
fclose(fid);
fprintf('Exported config to: %s\n', outFile);
fprintf('  N=%d  seed=%d  max_iter=%d\n', N, seed, max_iter);
