%MEASURE_LAG  How old is the camera image relative to the position input of the guidance block?
%   Flies the default track up to the first corner, then for every frame where the end of leg 1
%   is in view, projects the red pixels with the position from L frames back (L = 0..6) and
%   compares the projected line end with the true one (N = 2.55 in scene coordinates).
%   The L with the smallest error is the image lag.
here = fileparts(mfilename('fullpath')); root = fileparts(here);
pr = matlab.project.rootProject; if isempty(pr) || ~strcmp(pr.RootFolder, root), openProject(root); end
warning('off', 'all'); cw = onCleanup(@() warning('on', 'all'));
wp = [1.0 0.5; 1.0 2.5; 3.0 2.45]; ned = [wp(:,2) wp(:,1)];
D.posNED = [ned(1,:) - [0.5 1], -0.046]; D.Color = [1 0 0];
D.Lines = struct('StartX', num2cell(ned(1:end-1,1)'), 'StartY', num2cell(ned(1:end-1,2)'), ...
                 'EndX', num2cell(ned(2:end,1)'), 'EndY', num2cell(ned(2:end,2)'));
u = ned(end,:) - ned(end-1,:); u = u/norm(u); c = ned(end,:) + 0.3*u;
D.Circle = struct('X', c(1), 'Y', c(2), 'Z', 0.01, 'Diameter', 0.2);
save(fullfile(root, 'initData.mat'), '-struct', 'D');
top = 'PX4MultiCopterPerceptionAndControl';
load_system(top); load_system('PerceptionAndGuidance'); load_system('flightControlSystem');
if ~evalin('base', 'exist(''init'', ''var'') && exist(''MPC_XY_P'', ''var'')'), evalin('base', 'InitVars; setupParameters;'); end
init = evalin('base', 'init'); init.posNED = D.posNED; assignin('base', 'init', init);
out = sim(top, 'StopTime', '22');
sc = out.logsout.get('redScore').Values; Pv = out.logsout.get('dronePos').Values;
pd = squeeze(double(Pv.Data)); if size(pd,2) ~= 4, pd = pd'; end
nF = size(sc.Data, 3); tF = sc.Time; dtF = median(diff(tF));
fprintf('frames %d (dt %.3f s), pos samples %d (dt %.3f s)\n', nF, median(diff(tF)), size(pd,1), median(diff(Pv.Time)));
K08 = 0.0042; RC = 60.5; CC = 80.5; trueEnd = 2.55;
lags = 0:8; err = nan(nF, numel(lags));
for k = 1:nF
    m = sc.Data(:,:,k) > 50;
    for j = 1:numel(lags)
        tp = tF(k) - lags(j) * dtF;
        if tp < 0, continue; end
        q = interp1(Pv.Time, pd, tp, 'previous');          % pose L frames before the frame time
        h = -q(3); if h < 0.5, continue; end
        mpp = K08 * h / 0.8; [r, cc] = find(m); f = (RC - r)*mpp; s = (cc - CC)*mpp;
        N = q(1) + 0.5 + f*cos(q(4)) - s*sin(q(4)); E = q(2) + 1 + f*sin(q(4)) + s*cos(q(4));
        band = abs(E - 1.0) < 0.06;
        if nnz(band) < 50 || max(r(band)) - min(r(band)) < 10, continue; end
        nEnd = max(N(band));
        % only frames where the end is well inside the picture and we are moving
        v = norm(interp1(Pv.Time, pd(:,1:2), tF(k)) - interp1(Pv.Time, pd(:,1:2), max(tF(k)-0.4,0))) / 0.4;
        if min(r(band)) > 4 && v > 0.1, err(k,j) = nEnd - trueEnd; end
    end
end
ok = all(~isnan(err), 2);
fprintf('%d usable frames (moving, line end in view)\n', nnz(ok));
for j = 1:numel(lags)
    fprintf('lag %d frames (%.1f s): mean %+.3f m  rms %.3f m\n', lags(j), dtF*lags(j), mean(err(ok,j)), sqrt(mean(err(ok,j).^2)));
end
