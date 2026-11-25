%INSTALL_GUIDANCE  Make the lineGuidance MATLAB Function block call perception/trackGuidance.m.
%   Run once (and again if the block was edited by hand). Keeps the block's ports and logging.
root = fileparts(fileparts(mfilename('fullpath')));
pr = matlab.project.rootProject; if isempty(pr) || ~strcmp(pr.RootFolder, root)
    openProject(root);
end
load_system('PerceptionAndGuidance');
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', 'PerceptionAndGuidance/Image processing and Guidance/lineGuidance');
ch.Script = sprintf(['function [traj, dbg] = lineGuidance(mask, pos)\n' ...
    '%% Guidance lives in perception/trackGuidance.m (edit it there, then re-run the sim)\n' ...
    '[traj, dbg] = trackGuidance(mask, pos);\nend\n']);
save_system('PerceptionAndGuidance');
fprintf('lineGuidance block now calls trackGuidance.m\n');
