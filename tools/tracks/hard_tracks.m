function T = hard_tracks(N, seed)
% Generate N random tracks adhering to all geometric and obstacle rules.
if nargin < 2, seed = 0; end
rng(seed);
T = struct('name', {}, 'wp', {});
boxCenter = [2.25, 0.5];

for i = 1:N
    name = sprintf('rand%d', i);
    valid = false;
    for attempt = 1:200000
        nSec = randi([4, 7]);
        K = nSec + 1;
        wp = zeros(K, 2);
        
        % Start waypoint inside [0.4, 3.6] on both axes
        wp(1,:) = 0.4 + (3.6 - 0.4) .* rand(1, 2);
        
        % First section heading and length
        heading = rand * 360;
        length1 = 0.5 + (1.4 - 0.5) * rand;
        wp(2,:) = wp(1,:) + length1 * [cosd(heading), sind(heading)];
        
        fail = false;
        if any(wp(2,:) < 0.4 | wp(2,:) > 3.6)
            continue;
        end
        
        lengths = zeros(1, nSec);
        lengths(1) = length1;
        
        for k = 2:nSec
            turnMag = 90 + (145 - 90) * rand;
            if rand < 0.5
                turn = turnMag;
            else
                turn = -turnMag;
            end
            heading = heading + turn;
            
            secLen = 0.5 + (0.8 - 0.5) * rand;
            lengths(k) = secLen;
            
            wp(k+1,:) = wp(k,:) + secLen * [cosd(heading), sind(heading)];
            if any(wp(k+1,:) < 0.4 | wp(k+1,:) > 3.6)
                fail = true;
                break;
            end
        end
        
        if fail
            continue;
        end
        
        % Check waypoint bounds [0.4, 3.6]
        if any(wp(:) < 0.4 | wp(:) > 3.6)
            continue;
        end
        
        % Check section lengths [0.5, 1.4]
        if any(lengths < 0.5 | lengths > 1.4)
            continue;
        end
        
        % Check interior angles [35, 175] degrees
        ang = zeros(1, nSec - 1);
        angleFail = false;
        for k = 2:nSec
            a = wp(k-1,:) - wp(k,:);
            b = wp(k+1,:) - wp(k,:);
            na = norm(a); nb = norm(b);
            if na == 0 || nb == 0
                angleFail = true;
                break;
            end
            cosAngle = dot(a, b) / (na * nb);
            cosAngle = max(-1, min(1, cosAngle));
            ang(k-1) = acosd(cosAngle);
        end
        if angleFail || any(ang < 35 | ang > 175)
            continue;
        end
        
        % Check non-adjacent section distances (min distance >= 0.45 m)
        crossFail = false;
        for s1 = 1:nSec
            for s2 = (s1+2):nSec
                if segSegDist(wp(s1,:), wp(s1+1,:), wp(s2,:), wp(s2+1,:)) < 0.45
                    crossFail = true;
                    break;
                end
            end
            if crossFail
                break;
            end
        end
        if crossFail
            continue;
        end
        
        % Check landing circle: centre = last waypoint + 0.3 m along last section direction
        u_last = wp(end,:) - wp(end-1,:);
        u_last = u_last / norm(u_last);
        circ = wp(end,:) + 0.3 * u_last;
        
        if any(circ < 0.3 | circ > 3.7)
            continue;
        end
        
        circFail = false;
        for s = 1:(nSec - 1)
            if pointToSeg(circ, wp(s,:), wp(s+1,:)) < 0.45
                circFail = true;
                break;
            end
        end
        if circFail
            continue;
        end
        
        % Check scene squares: distance from segment to box centre (2.25, 0.5) >= 0.45 m
        boxFail = false;
        for s = 1:nSec
            if pointToSeg(boxCenter, wp(s,:), wp(s+1,:)) < 0.45
                boxFail = true;
                break;
            end
        end
        if boxFail
            continue;
        end
        
        valid = true;
        break;
    end
    
    if ~valid
        error('Failed to generate valid track %s after 5000 tries.', name);
    end
    
    T(end+1) = struct('name', name, 'wp', wp);
    fprintf('[rand] %s sections %d angles %s lengths %s\n', name, nSec, mat2str(round(ang)), mat2str(round(lengths, 2)));
end
end

function d = pointToSeg(P, A, B)
ab = B - A;
ab_sq = dot(ab, ab);
if ab_sq == 0
    d = norm(P - A);
else
    t = max(0, min(1, dot(P - A, ab) / ab_sq));
    d = norm(P - (A + t * ab));
end
end

function d = segSegDist(A1, A2, B1, B2)
if segmentsIntersect(A1, A2, B1, B2)
    d = 0;
else
    d = min([pointToSeg(A1, B1, B2), pointToSeg(A2, B1, B2), ...
             pointToSeg(B1, A1, A2), pointToSeg(B2, A1, A2)]);
end
end

function tf = segmentsIntersect(p1, p2, q1, q2)
d1 = direction(q1, q2, p1);
d2 = direction(q1, q2, p2);
d3 = direction(p1, p2, q1);
d4 = direction(p1, p2, q2);

if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
   ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
    tf = true;
elseif abs(d1) < 1e-9 && onSegment(q1, q2, p1)
    tf = true;
elseif abs(d2) < 1e-9 && onSegment(q1, q2, p2)
    tf = true;
elseif abs(d3) < 1e-9 && onSegment(p1, p2, q1)
    tf = true;
elseif abs(d4) < 1e-9 && onSegment(p1, p2, q2)
    tf = true;
else
    tf = false;
end
end

function d = direction(p, q, r)
d = (q(1) - p(1)) * (r(2) - p(2)) - (q(2) - p(2)) * (r(1) - p(1));
end

function tf = onSegment(p, q, r)
tf = r(1) >= min(p(1), q(1)) - 1e-9 && r(1) <= max(p(1), q(1)) + 1e-9 && ...
     r(2) >= min(p(2), q(2)) - 1e-9 && r(2) <= max(p(2), q(2)) + 1e-9;
end
