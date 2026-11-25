function R = fly_track(name, wpEN, opts)
%FLY_TRACK  Fly one track in the full Simulink + Unreal 3D model and score it.
%   R = fly_track('default', [1.0 0.5; 1.0 2.5; 3.0 2.45])
%   wpEN: waypoints [East North] exactly as the Track Builder app stores them.
%   opts.wind = [N E D] m/s, opts.stopTime (s, default from track length).
%   Saves logs/<name>.mat and figs/<name>.png next to this file.
if nargin < 3, opts = struct(); end
if isfile(fullfile(fileparts(mfilename('fullpath')), 'STOP')), error('fly_track:stop', 'STOP file present: batch stopped'); end
here = fileparts(mfilename('fullpath')); root = fileparts(here);
pr = matlab.project.rootProject; if isempty(pr) || ~strcmp(pr.RootFolder, root), openProject(root); end
top = 'PX4MultiCopterPerceptionAndControl';
ned = [wpEN(:,2) wpEN(:,1)];                     % scene [N E]
nS = size(ned,1) - 1;
u = ned(end,:) - ned(end-1,:); u = u / norm(u);
circ = ned(end,:) + u * 0.30;                    % Track Builder: w/2 + 0.25 past the end

% track file as the app writes it; the scene draws the drone at NED + (0.5, 1)
D.posNED = [ned(1,:) - [0.5 1], -0.046]; D.Color = [1 0 0];
D.Lines = struct('StartX', num2cell(ned(1:end-1,1)'), 'StartY', num2cell(ned(1:end-1,2)'), ...
                 'EndX', num2cell(ned(2:end,1)'), 'EndY', num2cell(ned(2:end,2)'));
D.Circle = struct('X', circ(1), 'Y', circ(2), 'Z', 0.01, 'Diameter', 0.2);
save(fullfile(root, 'initData.mat'), '-struct', 'D');

warning('off', 'all'); cw = onCleanup(@() warning('on', 'all'));   % hardware-board warnings are deploy-only
% Simulink does not always notice edits to trackGuidance.m (it is called from the block):
% stamp the block with a checksum of the file, so any edit changes the block and only it is rebuilt.
gsrc = fileread(fullfile(root, 'perception', 'trackGuidance.m'));
stamp = dec2hex(mod(sum(double(gsrc) .* (1:numel(gsrc))), 2^40), 10);
load_system('PerceptionAndGuidance');
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', 'PerceptionAndGuidance/Image processing and Guidance/lineGuidance');
want = sprintf(['function [traj, dbg] = lineGuidance(mask, pos)\n' ...
    '%% Guidance lives in perception/trackGuidance.m (version %s)\n' ...
    '[traj, dbg] = trackGuidance(mask, pos);\nend\n'], stamp);
if ~strcmp(ch.Script, want)
    ch.Script = want; save_system('PerceptionAndGuidance');
    fprintf('[%s] guidance version %s: block updated\n', name, stamp);
end
% project start-up variables (init, PX4 parameters) can be wiped by "clear": reload them if missing
if ~evalin('base', 'exist(''init'', ''var'') && exist(''MPC_XY_P'', ''var'')')
    evalin('base', 'InitVars; setupParameters;');
end
load_system(top); load_system('PerceptionAndGuidance'); load_system('flightControlSystem');
init = evalin('base', 'init'); init.posNED = D.posNED; assignin('base', 'init', init);
if isfield(opts, 'wind'), assignin('base', 'windVel', opts.wind(:)); end
L = sum(vecnorm(diff(ned), 2, 2));
stopT = 30 + 8 * L;                               % ~0.28 m/s plus takeoff, corners and landing
if isfield(opts, 'stopTime'), stopT = opts.stopTime; end

fprintf('[%s] flying %d sections (%.1f m), sim %.0f s ...\n', name, nS, L, stopT);
tw = tic;
for attempt = 1:3
    try
        out = sim(top, 'StopTime', num2str(stopT)); break
    catch e
        % the Unreal engine sometimes fails to start while the previous one is still closing
        if attempt == 3 || ~contains(e.message, {'3D', 'Simulation3D', 'sim3d'}), rethrow(e); end
        fprintf('[%s] 3D engine failed (%s), restarting it (try %d)\n', name, e.message(1:min(end,80)), attempt + 1);
        bdclose('all'); [~, ~] = system('taskkill /F /IM AutoVrtlEnv.exe'); pause(8);
        load_system(top); load_system('PerceptionAndGuidance'); load_system('flightControlSystem');
        init = evalin('base', 'init'); init.posNED = D.posNED; assignin('base', 'init', init);
        if isfield(opts, 'wind'), assignin('base', 'windVel', opts.wind(:)); end
    end
end
Pv = out.logsout.get('dronePos').Values; Gv = out.logsout.get('guidanceDbg').Values;
pd = squeeze(double(Pv.Data)); if size(pd,2) ~= 4, pd = pd'; end
gd = squeeze(double(Gv.Data)); if size(gd,2) ~= 4, gd = gd'; end
pos = pd(:,1:2) + [0.5 1];                        % scene coordinates

k0 = 1; done = 0;                                 % sections: vertices reached in order (0.15 m)
for i = 2:nS+1
    kk = find(vecnorm(pos(k0:end,:) - ned(i,:), 2, 2) < 0.15, 1);
    if isempty(kk), break; end
    k0 = k0 + kk - 1; done = done + 1;
end
e = inf(size(pos,1),1);
for i = 1:nS
    a = ned(i,:); b = ned(i+1,:); ab = b - a;
    tt = max(0, min(1, ((pos - a) * ab') / (ab*ab')));
    e = min(e, vecnorm(pos - (a + tt .* ab), 2, 2));
end
st = interp1(Gv.Time, gd(:,1), Pv.Time, 'previous', 'extrap');
fol = st == 1 | st == 2;
R = struct('name', name, 'sections', done, 'nSec', nS, 'meanErr', mean(e(fol)), 'maxErr', max(e(fol)), ...
    'landErr', norm(pos(end,:) - circ), 'landed', gd(end,1) == 7, 'simTime', stopT, 'wallTime', toc(tw), ...
    'pos', pos, 'alt', -pd(:,3), 'yaw', pd(:,4), 't', Pv.Time, 'gd', gd, 'gt', Gv.Time, 'ned', ned, 'circ', circ);
R.success = R.sections == nS && R.landed && R.landErr < 0.1;
fprintf('[%s] sections %d/%d  meanErr %.3f  maxErr %.3f  landErr %.3f  landed %d  -> %s  (%.0f s wall)\n', ...
    name, done, nS, R.meanErr, R.maxErr, R.landErr, R.landed, string(R.success), R.wallTime);
if ~isfolder(fullfile(here, 'logs')), mkdir(fullfile(here, 'logs')); end
save(fullfile(here, 'logs', [name '.mat']), 'R');
plot_track_flight(R, fullfile(here, 'figs', [name '.png']));
end

function plot_track_flight(R, file)
if ~isfolder(fileparts(file)), mkdir(fileparts(file)); end
names = {'TAKEOFF','FOLLOW','CORNER','COAST','SEARCH','APPROACH','DESCEND','LANDED'};
cols  = [.55 .55 .55; .12 .44 .82; .85 .2 .6; .94 .55 .1; .6 .2 .7; .1 .65 .65; .15 .6 .3; 0 0 0];
f = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1200 560]);
ax = subplot(2,2,[1 3]); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
plot(ax, R.ned(:,2), R.ned(:,1), '-', 'Color', [1 .55 .55], 'LineWidth', 9, 'HandleVisibility', 'off');
th = linspace(0, 2*pi, 60); plot(ax, R.circ(2) + 0.1*cos(th), R.circ(1) + 0.1*sin(th), 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
st = interp1(R.gt, R.gd(:,1), R.t, 'previous', 'extrap');
for s = 0:7, k = st == s; if any(k), plot(ax, R.pos(k,2), R.pos(k,1), '.', 'Color', cols(s+1,:), 'MarkerSize', 6, 'DisplayName', names{s+1}); end, end
legend(ax, 'show', 'Location', 'bestoutside', 'TextColor', 'k', 'Color', 'w');
xlim(ax, [0 4.5]); ylim(ax, [0 4.5]); xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)');
title(ax, sprintf('%s: %d/%d sections, landed %.1f cm off', R.name, R.sections, R.nSec, 100*R.landErr), 'Interpreter', 'none', 'Color', 'k');
ax2 = subplot(2,2,2); plot(ax2, R.t, R.alt, 'LineWidth', 1.5); grid(ax2, 'on'); ylabel(ax2, 'altitude (m)'); set(ax2, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
ax3 = subplot(2,2,4); stairs(ax3, R.gt, R.gd(:,1), 'LineWidth', 1.5); grid(ax3, 'on'); set(ax3, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
yticks(ax3, 0:7); yticklabels(ax3, names); ylim(ax3, [-0.5 7.5]); xlabel(ax3, 'time (s)');
exportgraphics(f, file, 'Resolution', 110); close(f);
end
