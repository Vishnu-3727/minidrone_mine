function txt = flight_story(R, file)
%FLIGHT_STORY  Plain-text account of one flight: every state change with time, place, speed and
%   heading; every corner (vertex reached? how close?); straightness per leg; landing; verdict and
%   any warnings (overshoot, search, false landing, wobble). Written to <file> if given.
names = {'TAKEOFF','FOLLOW','CORNER','COAST','SEARCH','APPROACH','DESCEND','LANDED'};
L = {};
L{end+1} = sprintf('FLIGHT %s   (%s)', R.name, char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')));
L{end+1} = sprintf('track: %d sections, %.2f m, angles %s deg, circle at N %.2f E %.2f', R.nSec, ...
    sum(vecnorm(diff(R.ned), 2, 2)), mat2str(round(track_angles(R.ned))), R.circ(1), R.circ(2));
L{end+1} = sprintf('result: %s | sections %d/%d | landed %d | land error %.1f cm | path error mean %.1f max %.1f cm', ...
    ternary(R.success, 'PASS', 'FAIL'), R.sections, R.nSec, R.landed, 100*R.landErr, 100*R.meanErr, 100*R.maxErr);
v = [0; vecnorm(diff(R.pos), 2, 2) ./ diff(R.t)];
st = interp1(R.gt, R.gd(:,1), R.t, 'previous', 'extrap');
kt = find(R.alt < 0.05 & R.t > 5, 1);
L{end+1} = sprintf('speed: top %.2f m/s, mean while following %.2f m/s | cruise altitude %.2f m | touchdown %s', ...
    max(v), mean(v(st == 1)), median(R.alt(st == 1)), ternary(isempty(kt), 'never', sprintf('%.1f s', R.t(kt))));
L{end+1} = '';
L{end+1} = 'TIMELINE';
g = R.gd(:,1); ch = [1; find(diff(g)) + 1];
for k = ch.'
    j = find(R.t <= R.gt(k), 1, 'last'); if isempty(j), j = 1; end
    hd = R.gd(k,4); hs = ternary(hd == 99, '   -  ', sprintf('%6.1f', hd*180/pi));
    L{end+1} = sprintf('  %6.1f s  %-8s  at N %.2f E %.2f  alt %.2f  speed %.2f  heading %s', ...
        R.gt(k), names{g(k)+1}, R.pos(j,1), R.pos(j,2), R.alt(j), v(j), hs); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = 'CORNERS (closest approach to each vertex, in order)';
k0 = 1;
for i = 2:R.nSec
    d = vecnorm(R.pos(k0:end,:) - R.ned(i,:), 2, 2); [dm, km] = min(d); km = km + k0 - 1;
    a = R.ned(i-1,:) - R.ned(i,:); b = R.ned(i+1,:) - R.ned(i,:);
    % overshoot: how far past the vertex along the incoming leg the drone went
    u = -a / norm(a); ov = max(((R.pos(max(km-15,1):min(km+15,end),:) - R.ned(i,:)) * u.'));
    L{end+1} = sprintf('  vertex %d (N %.2f E %.2f, %3.0f deg): closest %.1f cm at %.1f s, overshoot %.1f cm%s', i-1, ...
        R.ned(i,1), R.ned(i,2), acosd(dot(a,b)/norm(a)/norm(b)), 100*dm, R.t(km), 100*max(ov,0), ...
        ternary(dm > 0.15, '   <-- MISSED', ternary(ov > 0.10, '   <-- overshoot', ''))); %#ok<AGROW>
    k0 = km;
end
L{end+1} = '';
L{end+1} = 'STRAIGHTNESS PER LEG (straight flight only)';
Q = evalc('Qt = flight_quality(R);'); %#ok<NASGU>
for i = 1:height(Qt)
    L{end+1} = sprintf('  leg %d: sway %.1f cm (p2p %.1f), yaw wobble %.1f deg (p2p %.1f)%s', Qt.leg(i), Qt.sway_std_cm(i), ...
        Qt.sway_p2p_cm(i), Qt.yaw_std_deg(i), Qt.yaw_p2p_deg(i), ternary(Qt.sway_p2p_cm(i) > 5 || Qt.yaw_p2p_deg(i) > 8, '   <-- wobble', '')); %#ok<AGROW>
end
L{end+1} = '';
W = {};
if any(g == 4), W{end+1} = 'SEARCH used (line lost)'; end
if R.landed && R.landErr > 0.1, W{end+1} = 'FALSE LANDING: landed away from the circle'; end
if ~R.landed, W{end+1} = 'did not land before the time limit'; end
if R.maxErr > 0.12, W{end+1} = sprintf('path error %.0f cm', 100*R.maxErr); end
L{end+1} = ['WARNINGS: ' ternary(isempty(W), 'none', strjoin(W, '; '))];
txt = strjoin(L, newline);
if nargin > 1
    fid = fopen(file, 'w'); fprintf(fid, '%s\n', txt); fclose(fid);
end
end

function a = track_angles(ned)
a = zeros(1, size(ned,1) - 2);
for i = 2:size(ned,1) - 1
    p = ned(i-1,:) - ned(i,:); q = ned(i+1,:) - ned(i,:); a(i-1) = acosd(dot(p,q)/norm(p)/norm(q));
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
