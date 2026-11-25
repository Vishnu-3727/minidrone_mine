function auto_tune(validate)
%AUTO_TUNE  Unattended tuning of trackGuidance.m in the 3D model, then full validation.
%   auto_tune          tune (coordinate search, 1 round, ~1.5 h) then fly the full 103-track suite (~1.3 h)
%   auto_tune(false)   tune only
%   Each candidate value of one parameter is written into perception/trackGuidance.m and flown on a
%   10-track training set. A candidate is kept only if its score beats the best by > MARGIN and it adds
%   no lost points. Score = 1000 * points/maxPoints - 0.5 * mean time - 0.5 * mean max path error (cm)
%   - 0.3 * mean max sway (cm); points as in the competition: take-off 1, each section 1, soft landing 1,
%   accurate landing 1 (landing error < 10 cm).
%   State is saved after every evaluation in logs/tune/state.mat, so a re-run resumes where it stopped.
%   Log of every evaluation: logs/tune/tune_log.csv. Report: logs/tune/REPORT.md.
if nargin < 1, validate = true; end
here = fileparts(mfilename('fullpath')); root = fileparts(here);
gfile = fullfile(root, 'perception', 'trackGuidance.m');
td = fullfile(here, 'logs', 'tune'); if ~isfolder(td), mkdir(td); end
stFile = fullfile(td, 'state.mat'); logCsv = fullfile(td, 'tune_log.csv');
MARGIN = 2;
TRAIN = {'default','w31','zigzag7','hard_s1_1','hard_s2_3','vhard_s1_1','my_track10','cross_s1_1'};   % 8 tracks, ~7 min per evaluation
% parameter, candidate multipliers of the current value, [min max]
P = {'A_UP',       [0.6 1.5],  [0.3 1.5];
     'A_BRAKE',    [0.7 1.3],  [0.6 2.5];
     'LAG_T',      [0.7 1.3],  [0.15 0.5];
     'END_MARGIN', [0.7 1.4],  [0.05 0.16];
     'LOOK_R',     [0.85 1.12],[0.18 0.26];
     'YAW_ACC',    [0.6 1.5],  [40*pi/180 300*pi/180]};
src0 = fileread(gfile);
if isfile(stFile)
    S = load(stFile); S = S.S; fprintf('auto_tune: resuming at round %d, param %d\n', S.round, S.k);
else
    S.best = getVals(src0, P(:,1)); S.bestScore = NaN; S.bestPts = NaN; S.round = 1; S.k = 1; S.n = 0;
    S.history = {};
    fid = fopen(logCsv, 'w'); fprintf(fid, 'eval,round,param,value,score,points,maxPoints,meanTime,meanMaxErr,meanSway,accepted,tag\n'); fclose(fid);
    copyfile(gfile, fullfile(td, 'trackGuidance_start.m.txt'));
end
if isnan(S.bestScore)                                   % baseline
    [S.bestScore, S.bestPts, m] = evalSet(S.best, 'base');
    logRow(0, 'base', NaN, S.bestScore, m, true, 'base'); save(stFile, 'S');
end
NROUND = 1;                                             % 3 h budget: 1 round + full validation
for r = S.round:NROUND
    for k = S.k:size(P,1)
        name = P{k,1}; cur = S.best.(name);
        for mlt = P{k,2}
            v = min(max(cur*mlt, P{k,3}(1)), P{k,3}(2));
            if abs(v - cur) < 1e-6, continue; end
            cand = S.best; cand.(name) = v;
            S.n = S.n + 1; tag = sprintf('e%02d_%s_%g', S.n, name, round(v, 4));
            [sc, pts, m] = evalSet(cand, tag);
            ok = sc > S.bestScore + MARGIN && pts >= S.bestPts;
            logRow(r, name, v, sc, m, ok, tag);
            fprintf('auto_tune: %s = %.4g -> score %.1f (best %.1f) %s\n', name, v, sc, S.bestScore, ternary(ok, 'ACCEPTED', 'rejected'));
            if ok, S.best = cand; S.bestScore = sc; S.bestPts = pts; cur = v; end
            S.round = r; S.k = k; save(stFile, 'S');
        end
    end
    S.k = 1;
end
S.round = 3; save(stFile, 'S');
writeVals(S.best);                                      % leave the winner in the file
fprintf('auto_tune: best score %.1f with %s\n', S.bestScore, jsonencode(S.best));
if validate
    names = strtrim(splitlines(fileread(fullfile(here, 'tracks', 'legal_names.txt')))); names = names(~cellfun(@isempty, names)).';
    suite_run([names, arrayfun(@(i) sprintf('cross_s1_%d', i), 1:6, 'UniformOutput', false), ...
               arrayfun(@(i) sprintf('cross_s2_%d', i), 1:6, 'UniformOutput', false)], 'final');
end
report(S, validate);

    % ---------------------------------------------------------------------------------------------
    function [sc, pts, m] = evalSet(vals, tag)
        if isfile(fullfile(here, 'STOP')), error('auto_tune:stop', 'STOP file present: tuner stopped (re-run auto_tune to resume)'); end
        writeVals(vals);
        suite_run(TRAIN, fullfile('tune', tag));
        [sc, pts, m] = scoreCsv(fullfile(here, 'logs', 'tune', tag, 'results.csv'));
    end
    function writeVals(vals)
        s = fileread(gfile); f = fieldnames(vals);
        for i = 1:numel(f)
            assert(~isempty(regexp(s, ['(?m)^' f{i} '\s*='], 'once')), 'parameter %s not found', f{i});
            s = regexprep(s, ['(?m)^(' f{i} '\s*=\s*)[^;]+;'], ['$1' sprintf('%.6g', vals.(f{i})) ';'], 'once');
        end
        fid = fopen(gfile, 'w'); fwrite(fid, s); fclose(fid);
    end
    function logRow(r, name, v, sc, m, ok, tag)
        fid = fopen(logCsv, 'a');
        fprintf(fid, '%d,%d,%s,%.6g,%.2f,%d,%d,%.2f,%.2f,%.2f,%d,%s\n', S.n, r, name, v, sc, m.pts, m.maxPts, m.time, m.err, m.sway, ok, tag);
        fclose(fid);
        S.history(end+1, :) = {r, name, v, sc, ok}; %#ok<SETNU>
    end
end

function v = getVals(src, names)
v = struct();
for i = 1:numel(names)
    t = regexp(src, ['(?m)^' names{i} '\s*=\s*([^;]+);'], 'tokens', 'once');
    v.(names{i}) = eval(t{1});
end
end

function [sc, pts, m] = scoreCsv(csv)
D = readtable(csv, 'Delimiter', ',', 'TextType', 'string');
took = D.sections > 0 | D.landed > 0;
p = double(took) + D.sections + double(D.landed) + double(D.landed & D.landErr_cm < 10);
mx = D.nSec + 3; mx(mx < 3) = 3;
td = D.touchdown_s; td(isnan(td)) = 120;
e = D.maxErr_cm; e(isnan(e)) = 50; w = D.maxSway_cm; w(isnan(w)) = 20;
m.pts = sum(p); m.maxPts = sum(mx); m.time = mean(td); m.err = mean(e); m.sway = mean(w);
pts = m.pts;
sc = 1000 * m.pts / m.maxPts - 0.5*m.time - 0.5*m.err - 0.3*m.sway;
end

function report(S, validated)
here = fileparts(mfilename('fullpath')); td = fullfile(here, 'logs', 'tune');
L = {'# Auto-tune report', '', sprintf('Generated %s', char(datetime('now'))), ''};
L{end+1} = sprintf('Best training score: %.1f (points %d)', S.bestScore, S.bestPts);
L{end+1} = ''; L{end+1} = '## Final parameters'; f = fieldnames(S.best);
for i = 1:numel(f), L{end+1} = sprintf('- `%s = %.6g`', f{i}, S.best.(f{i})); end %#ok<AGROW>
L{end+1} = ''; L{end+1} = '## Every evaluation'; L{end+1} = '';
L{end+1} = '| round | parameter | value | score | kept |'; L{end+1} = '|---|---|---|---|---|';
for i = 1:size(S.history, 1)
    L{end+1} = sprintf('| %d | %s | %.4g | %.1f | %s |', S.history{i,1}, S.history{i,2}, S.history{i,3}, S.history{i,4}, ternary(S.history{i,5}, 'yes', '')); %#ok<AGROW>
end
if validated
    D = readtable(fullfile(here, 'logs', 'final', 'results.csv'), 'Delimiter', ',', 'TextType', 'string');
    L{end+1} = ''; L{end+1} = '## Full validation (logs/final)';
    L{end+1} = sprintf('%d / %d flights pass (all sections + landed within 10 cm)', nnz(D.pass), height(D));
    L{end+1} = sprintf('landing error: mean %.2f cm, max %.2f cm', mean(D.landErr_cm, 'omitnan'), max(D.landErr_cm));
    L{end+1} = sprintf('max path error: mean %.1f cm, worst %.1f cm', mean(D.maxErr_cm, 'omitnan'), max(D.maxErr_cm));
    L{end+1} = sprintf('touchdown time: mean %.1f s', mean(D.touchdown_s, 'omitnan'));
    F = D(D.pass == 0, :);
    if height(F) > 0
        L{end+1} = ''; L{end+1} = 'Failed flights (story logs in logs/final/<track>.txt):';
        for i = 1:height(F), L{end+1} = sprintf('- %s: sections %d/%d, landed %d, %s', F.track(i), F.sections(i), F.nSec(i), F.landed(i), F.warnings(i)); end %#ok<AGROW>
    end
end
fid = fopen(fullfile(td, 'REPORT.md'), 'w'); fprintf(fid, '%s\n', strjoin(L, newline)); fclose(fid);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
