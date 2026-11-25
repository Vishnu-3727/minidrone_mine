function T = legal_tracks(names)
%LEGAL_TRACKS  The team's 91-track legal acceptance suite (tracks/legal_names.txt), [East North].
%   T = legal_tracks()            all 91
%   T = legal_tracks({'default','w31'})
%   Same definitions as the team harness (tva5.m): core tracks, random_tracks seeds 1-8,
%   hard_tracks seeds 1-3, vhard_tracks seeds 1-2. Rule: 2-7 sections, every angle > 30 deg.
d = fullfile(fileparts(mfilename('fullpath')), 'tracks'); addpath(d);
A = struct('name', {}, 'wp', {});
W = {'sharp40',[0.5 0.5; 0.5 3.0; 1.79 1.47]; 'southStart60',[3.5 3.5; 3.5 1.0; 1.33 2.25]; ...
     'pdfSample',[2.0 0.5; 2.0 2.0; 3.25 3.0]; 'zigzag7',[0.4 0.4; 0.4 1.6; 1.2 1.0; 1.6 2.4; 2.4 1.4; 3.4 1.6; 3.0 3.0; 1.8 3.4]; ...
     'sharp33',[3.5 1.0; 1.0 1.0; 3.1 2.36]; 'default',[1.0 0.5; 1.0 2.5; 3.0 2.45]; ...
     'vtrack',[0.4 0.9; 1.1 3.4; 1.9 0.9; 2.7 3.4; 3.4 1.0]; 'vtrackRot',[0.9 0.4; 3.4 1.1; 0.9 1.9; 3.4 2.7; 1.0 3.4]; ...
     'v31',[0.5 3.4; 1.1 0.9; 1.9 3.4; 2.7 0.9; 3.5 3.3]; ...
     'video',[2.03 0.71; 1.09 1.24; 1.09 2.46; 2.12 2.88; 2.42 1.82; 2.95 2.07]; ...
     'v30',[1.0 0.6; 1.0 3.2; 2.37 0.98]; 'eastwall',[2.0 2.5; 2.62 3.71; 3.58 2.39]; ...
     'w31',[0.5 3.0; 1.2 0.4; 1.9 3.0; 2.6 0.4; 3.3 3.0]};
for i = 1:size(W,1), A(end+1) = struct('name', W{i,1}, 'wp', W{i,2}); end %#ok<AGROW>
quiet = evalc('R1 = arrayfun(@(s) random_tracks(6, s), 1:8, ''UniformOutput'', false);'); %#ok<NASGU>
for s = 1:8, for i = 1:numel(R1{s}), A(end+1) = struct('name', sprintf('rand_s%d_%d', s, i), 'wp', R1{s}(i).wp); end, end %#ok<AGROW>
quiet = evalc('R2 = arrayfun(@(s) hard_tracks(6, s), 1:3, ''UniformOutput'', false);'); %#ok<NASGU>
for s = 1:3, for i = 1:numel(R2{s}), A(end+1) = struct('name', sprintf('hard_s%d_%d', s, i), 'wp', R2{s}(i).wp); end, end %#ok<AGROW>
quiet = evalc('R3 = arrayfun(@(s) vhard_tracks(6, s), 1:2, ''UniformOutput'', false);'); %#ok<NASGU>
for s = 1:2, for i = 1:numel(R3{s}), A(end+1) = struct('name', sprintf('vhard_s%d_%d', s, i), 'wp', R3{s}(i).wp); end, end %#ok<AGROW>
% team stress maps (CSV) and generated crossing-heavy maps
f = [dir(fullfile(d, 'my_track*.csv')); dir(fullfile(d, 'new_map_*.csv'))];
for i = 1:numel(f), A(end+1) = struct('name', erase(f(i).name, '.csv'), 'wp', readmatrix(fullfile(d, f(i).name))); end %#ok<AGROW>
for s = 1:2
    C = cross_tracks(6, s, 3);
    for i = 1:numel(C), A(end+1) = struct('name', C(i).name, 'wp', C(i).wp); end %#ok<AGROW>
end
if nargin < 1
    names = strtrim(splitlines(fileread(fullfile(d, 'legal_names.txt')))); names = names(~cellfun(@isempty, names));
end
[ok, idx] = ismember(names, {A.name});
if any(~ok), warning('unknown tracks: %s', strjoin(names(~ok), ', ')); end
T = A(idx(ok));
end
