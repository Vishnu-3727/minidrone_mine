%RUN_SUITE  Fly a list of tracks in the 3D model, write a story log per flight and a results table.
%   SUITE = {'default','sharp33'}; TAG = 'v3'; run_suite     (SUITE omitted = all 91 legal tracks)
%   Logs:    tools/logs/<TAG>/<track>.txt (story), .mat (data), figs/<TAG>/<track>.png
%   Results: tools/logs/<TAG>/results.csv (one row per flight, appended as flights finish)
here = fileparts(mfilename('fullpath')); addpath(here);
if ~exist('TAG', 'var'), TAG = char(datetime('now', 'Format', 'MMdd_HHmm')); end
if exist('SUITE', 'var'), T = legal_tracks(SUITE); else, T = legal_tracks(); end
ld = fullfile(here, 'logs', TAG); if ~isfolder(ld), mkdir(ld); end
csv = fullfile(ld, 'results.csv');
if ~isfile(csv)
    fid = fopen(csv, 'w');
    fprintf(fid, 'track,pass,sections,nSec,landed,landErr_cm,meanErr_cm,maxErr_cm,topSpeed,meanSpeed,touchdown_s,maxSway_cm,maxYawWobble_deg,warnings\n');
    fclose(fid);
end
gsrc = fileread(fullfile(fileparts(here), 'perception', 'trackGuidance.m'));
fprintf('suite %s: %d tracks, guidance hash %s\n', TAG, numel(T), dec2hex(sum(double(gsrc) .* (1:numel(gsrc))), 10));
done = {};                                  % resume: skip tracks that already have a (non-STOP) result
if isfile(csv)
    D = readtable(csv, 'Delimiter', ',', 'TextType', 'string');
    if height(D) > 0, done = cellstr(D.track(~contains(D.warnings, 'STOP file'))); end
end
for i = 1:numel(T)
    if ismember(T(i).name, done), continue; end
    try
        try
            R = fly_track(T(i).name, T(i).wp);
        catch e1
            if contains(e1.message, 'STOP file'), rethrow(e1); end
            fprintf('[%s] error (%s): retrying once\n', T(i).name, e1.message); bdclose('all');
            R = fly_track(T(i).name, T(i).wp);
        end
        movefile(fullfile(here, 'logs', [T(i).name '.mat']), fullfile(ld, [T(i).name '.mat']));
        fd = fullfile(here, 'figs', TAG); if ~isfolder(fd), mkdir(fd); end
        movefile(fullfile(here, 'figs', [T(i).name '.png']), fullfile(fd, [T(i).name '.png']));
        txt = flight_story(R, fullfile(ld, [T(i).name '.txt']));
        Q = evalc('Qt = flight_quality(R);'); %#ok<NASGU>
        v = [0; vecnorm(diff(R.pos), 2, 2) ./ diff(R.t)];
        st = interp1(R.gt, R.gd(:,1), R.t, 'previous', 'extrap');
        kt = find(R.alt < 0.05 & R.t > 5, 1); if isempty(kt), td = NaN; else, td = R.t(kt); end
        w = regexp(txt, 'WARNINGS: (.*)$', 'tokens', 'once', 'lineanchors');
        fid = fopen(csv, 'a');
        fprintf(fid, '%s,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,"%s"\n', T(i).name, R.success, R.sections, R.nSec, ...
            R.landed, 100*R.landErr, 100*R.meanErr, 100*R.maxErr, max(v), mean(v(st == 1)), td, ...
            max([Qt.sway_p2p_cm; 0]), max([Qt.yaw_p2p_deg; 0]), w{1});
        fclose(fid);
    catch e
        fid = fopen(csv, 'a'); fprintf(fid, '%s,0,0,0,0,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,"ERROR %s"\n', T(i).name, strrep(e.message, '"', '''')); fclose(fid);
        fprintf('[%s] ERROR %s\n', T(i).name, e.message);
        bdclose('all');
    end
end
S = readtable(csv);
fprintf('\nsuite %s: %d / %d pass | landErr max %.1f cm | path max %.1f cm | mean speed %.2f m/s\n', TAG, ...
    nnz(S.pass), height(S), max(S.landErr_cm), max(S.maxErr_cm), mean(S.meanSpeed, 'omitnan'));
