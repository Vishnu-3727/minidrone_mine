function Q = flight_quality(R)
%FLIGHT_QUALITY  How straight and steady a flight was: per leg, ignoring the first 0.3 m after
%   each corner and the last 0.2 m, and the corner pivot itself. Prints and returns a table.
y = unwrap(R.yaw) * 180/pi;
st = interp1(R.gt, R.gd(:,1), R.t, 'previous', 'extrap');
% each sample belongs to its nearest leg (legs of a sharp V run side by side near the vertex)
dSeg = inf(size(R.pos,1), R.nSec);
for i = 1:R.nSec
    a = R.ned(i,:); b = R.ned(i+1,:); ab = b - a; tt = max(0, min(1, ((R.pos - a) * ab') / (ab*ab')));
    dSeg(:,i) = vecnorm(R.pos - (a + tt .* ab), 2, 2);
end
[~, near] = min(dSeg, [], 2);
rows = {};
for i = 1:R.nSec
    a = R.ned(i,:); b = R.ned(i+1,:); L = norm(b - a); u = (b - a) / L;
    s = (R.pos - a) * u.'; l = (R.pos - a) * [-u(2) u(1)].';
    k = s > 0.3 & s < L - 0.2 & abs(l) < 0.2 & near == i & st == 1;   % straight flight only (not the pivot)
    if nnz(k) < 2, continue; end
    yaw0 = atan2d(u(2), u(1));
    rows(end+1,:) = {i, nnz(k), 100*std(l(k)), 100*(max(l(k)) - min(l(k))), ...
        std(y(k)), max(y(k)) - min(y(k)), mean(mod(y(k) - yaw0 + 180, 360) - 180)}; %#ok<AGROW>
end
vn = {'leg','samples','sway_std_cm','sway_p2p_cm','yaw_std_deg','yaw_p2p_deg','yaw_bias_deg'};
if isempty(rows), Q = cell2table(cell(0, numel(vn)), 'VariableNames', vn); else, Q = cell2table(rows, 'VariableNames', vn); end
disp(Q);
end
