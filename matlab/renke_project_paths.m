function paths = renke_project_paths()
%RENKE_PROJECT_PATHS Resolve repo-local paths for MATLAB demos.
%   Keeps outputs/references robust when the repository is moved.

thisFile = mfilename('fullpath');
if isempty(thisFile)
    error('renke_project_paths:context', ...
          'This helper must be called from a MATLAB file in the repo.');
end

thisDir = fileparts(thisFile); % .../matlab

% Find repository root by locating .git. When downloaded as a ZIP, .git is
% absent, so fall back to the known repo/matlab layout.
repoRoot = fileparts(thisDir);
probeDir = thisDir;
while ~isempty(probeDir)
    if isfolder(fullfile(probeDir, '.git'))
        repoRoot = probeDir;
        break;
    end
    parent = fileparts(probeDir);
    if isempty(parent) || strcmp(parent, probeDir)
        break;
    end
    probeDir = parent;
end

paths = struct();
paths.thisDir = thisDir;
paths.repoRoot = repoRoot;
paths.demoRoot = fullfile(repoRoot, 'traffic-demo');
paths.batchDir = fullfile(thisDir, 'BatchRuns');
paths.demoDir = fullfile(paths.demoRoot, 'schedules');
paths.demoImageDir = fullfile(paths.demoDir, 'images');
end
