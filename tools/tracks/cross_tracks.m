function T = cross_tracks(N, seed, minCross)
%CROSS_TRACKS  Random legal tracks with many X-crossings, [East North] waypoints.
%   T = cross_tracks(6, 1, 3)   6 tracks, seed 1, at least 3 crossings each
%   Rules kept: 4-7 sections, every interior angle > 35 deg (margin over the 30 deg limit),
%   legs 0.6-2.5 m, waypoints and circle inside 0.4..3.6 m, crossings at > 35 deg (no near-
%   parallel overlap), every crossing and vertex >= 0.25 m from any other vertex, and the
%   landing circle >= 0.3 m from every leg it does not belong to.
if nargin < 3, minCross = 3; end
rng(seed);
T = struct('name', {}, 'wp', {}, 'nCross', {});
while numel(T) < N
    nS = randi([4 7]);
    wp = zeros(nS+1, 2); wp(1,:) = 0.4 + 3.2*rand(1,2);
    hd = 360*rand; ok = true;
    for k = 1:nS
        placed = false;
        for tries = 1:200
            if k > 1, h = hd + (145*rand) * sign(randn); else, h = hd; end   % turn <= 145 deg => angle >= 35
            L = 0.6 + 1.9*rand;
            q = wp(k,:) + L*[cosd(h) sind(h)];
            if all(q >= 0.4 & q <= 3.6), wp(k+1,:) = q; hd = h; placed = true; break; end
        end
        if ~placed, ok = false; break; end
    end
    if ~ok, continue; end
    [good, nc] = check(wp);
    if good && nc >= minCross
        T(end+1) = struct('name', sprintf('cross_s%d_%d', seed, numel(T)+1), 'wp', wp, 'nCross', nc); %#ok<AGROW>
    end
end
end

function [good, nc] = check(wp)
good = false; nc = 0; n = size(wp,1) - 1;
for k = 2:n                                         % interior angles
    a = wp(k-1,:) - wp(k,:); b = wp(k+1,:) - wp(k,:);
    if acosd(dot(a,b)/norm(a)/norm(b)) <= 35, return; end
end
u = wp(end,:) - wp(end-1,:); c = wp(end,:) + 0.30*u/norm(u);
if any(c < 0.4 | c > 3.6), return; end
for i = 1:n
    for j = i+2:n                                   % non-adjacent legs
        [hit, P] = segX(wp(i,:), wp(i+1,:), wp(j,:), wp(j+1,:));
        if hit
            a = wp(i+1,:) - wp(i,:); b = wp(j+1,:) - wp(j,:);
            ang = acosd(abs(dot(a,b))/norm(a)/norm(b));
            if ang < 35, return; end                % near-parallel crossing
            if min(vecnorm(wp - P, 2, 2)) < 0.25, return; end   % crossing too close to a vertex
            nc = nc + 1;
        else
            if segDist(wp(i,:), wp(i+1,:), wp(j,:), wp(j+1,:)) < 0.25, return; end  % near miss
        end
    end
end
for k = 1:n+1                                       % vertex next to a foreign leg
    for i = 1:n
        if i == k || i == k-1, continue; end
        if pointSeg(wp(k,:), wp(i,:), wp(i+1,:)) < 0.25, return; end
    end
end
for i = 1:n-1                                       % circle clear of other legs
    if pointSeg(c, wp(i,:), wp(i+1,:)) < 0.30, return; end
end
good = true;
end

function [hit, P] = segX(p1, p2, q1, q2)
r = p2 - p1; s = q2 - q1; den = r(1)*s(2) - r(2)*s(1); P = [NaN NaN]; hit = false;
if abs(den) < 1e-12, return; end
t = ((q1(1)-p1(1))*s(2) - (q1(2)-p1(2))*s(1)) / den;
u = ((q1(1)-p1(1))*r(2) - (q1(2)-p1(2))*r(1)) / den;
if t > 0 && t < 1 && u > 0 && u < 1, hit = true; P = p1 + t*r; end
end

function d = pointSeg(p, a, b)
ab = b - a; t = max(0, min(1, dot(p - a, ab)/dot(ab, ab))); d = norm(p - (a + t*ab));
end

function d = segDist(a1, a2, b1, b2)
d = min([pointSeg(a1, b1, b2), pointSeg(a2, b1, b2), pointSeg(b1, a1, a2), pointSeg(b2, a1, a2)]);
end
