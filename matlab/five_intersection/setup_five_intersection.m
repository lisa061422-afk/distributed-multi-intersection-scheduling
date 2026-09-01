function paths = setup_five_intersection()
%SETUP_FIVE_INTERSECTION Add the five-intersection module and shared code.
%   Resolves every folder from this file so the repository remains portable.

moduleDir = fileparts(mfilename('fullpath'));
sharedSchedulerDir = fileparts(moduleDir);

addpath(moduleDir);
addpath(sharedSchedulerDir);

paths = renke_project_paths();
paths.fiveIntersectionDir = moduleDir;
paths.sharedSchedulerDir = sharedSchedulerDir;

% Optional override used by smoke tests or callers that want disposable
% results without changing the preserved BatchRuns examples.
outputOverride = getenv('RENKE_FIVE_INT_OUTPUT_DIR');
if ~isempty(outputOverride)
    paths.batchDir = outputOverride;
end
end
