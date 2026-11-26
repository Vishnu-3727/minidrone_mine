function [traj, dbg] = trackGuidance(mask, pos)
%#codegen
% Line-following guidance for the PX4 minidrone (MATLAB Function block "lineGuidance").
% Same interface as the team model: mask 120x160 logical (true = red), pos = [x y z yaw] NED.
% traj = [x_b; y_b; z; yaw] as localToNED expects (target NED pre-rotated by the yaw command).
% dbg  = [state; yawCmd; linePixels; heading or 99].
%
% Every pixel is projected onto the floor, so all thresholds are in metres and do not
% depend on altitude. States: 0 TAKEOFF 1 FOLLOW 2 CORNER 3 COAST 4 SEARCH 5 APPROACH 6 DESCEND 7 LANDED

%% ---------------- parameters (all in metres / radians unless noted) ----------------
ALT        = 1.0;          % flight altitude (footprint 0.84 x 0.63 m, line 19 px): sees 0.31 m ahead
K08        = 0.0042;       % camera scale measured at 0.8 m, m/px (Unreal camera)
RC = 60.5; CC = 80.5;      % optical centre (row, col)
FRAME_DT   = 0.1;          % guidance frame period, s (10 Hz: camera + decision every 0.1 s)
IMG_AGE    = 0.2;          % s, image age relative to the position input (measured: tools/measure_lag.m)
LAG_POS    = 2;            % = IMG_AGE / FRAME_DT frames (literal: codegen needs a constant size)
LAG_YAW    = 2;
RED_MIN_PX = 30;           % ignore red specks smaller than this
LINE_LEN   = 0.35;         % blob longer than this (or border blob longer than 0.22) is line
CIRC_D     = 0.20;         % landing circle diameter
CIRC_TOL   = 0.40;         % circle area within +-40 %
CIRC_ROUND = 0.80;         % circle minor/major spread
CIRC_FR    = round(0.6/FRAME_DT);   % circle seen for 0.6 s in a row
CIRC_JUMP  = 0.08;         % consecutive circle fixes agree within this
LOOK_R     = 0.187;         % look-ahead ring radius on the floor (image half-height = 0.31 m at 1.0 m)
RING_HALF  = 0.025;        % ring half thickness (5 px at 1 m)
NB         = 72;           % ring bins
OCC_FRAC   = 0.30;         % bin occupied above this red fraction
FAT_BINS   = 8;            % wider crossing = two legs merging (single line ~5-6 bins)
AHEAD      = 45*pi/180;    % crossing within this of the heading continues the line ...
AHEAD_STR  = 20*pi/180;    % ... or within this when more than one way on is visible
BACK_ZONE  = 25*pi/180;    % crossing within this of the breadcrumb is where we came from
CRUMB_ARC  = 0.25;         % breadcrumb distance back along the flown path
OLD_TOL    = 0.07;         % crossing this close to any flown point = behind us / old leg
JOIN_FRAC  = 0.85;         % way on must be joined to us by a straight red strip
STEP_MAX   = 0.30;         % carrot used off the line-fit path (crossing pursuit)
V_MAX      = 1.0;          % top speed on a long straight, m/s
A_BRAKE    = 1.5;          % braking we plan with, m/s^2: the CORNER profile commands it (PX4 limit MPC_ACC_HOR 3)
LAG_T      = 0.3;          % s, image age + PX4 reaction: flown blind before braking starts
KP_XY      = 0.95;         % PX4 MPC_XY_P: velocity asked = KP_XY * setpoint distance
KV_XY      = 1.8;          % PX4 MPC_XY_VEL_P_ACC: acceleration = KV_XY * velocity error
A_MAX      = 2.5;          % never ask for harder braking than this
% output shaper: every mode change goes through it, so speed, turns, stops and height change smoothly
A_UP       = 0.6;          % m/s^2, fastest speed-up (setpoint lead grows at most A_UP/KP_XY per s)
A_DOWN     = 2.5;          % m/s^2, fastest slow-down / direction change (never weaker than the braking)
YAW_RATE   = 60*pi/180;    % rad/s, top turn rate
YAW_ACC    = 2.0944;   % rad/s^2, turn rate ramps up and down (no snap at start / end of a turn)
VZ_MAX     = 0.6;          % m/s, top climb / sink rate of the height setpoint
AZ_MAX     = 0.5;          % m/s^2, climb / sink rate ramps (soft lift-off, soft level-off, soft touchdown)
END_MARGIN = 0.1;         % stop this far before the visible line end (a vertex is confirmed only 8 cm inside the view)
T_CARROT   = 1.1;          % s, carrot distance = speed * this (PX4 MPC_XY_P = 0.95 /s)
STEP_MIN   = 0.06;         % carrot while still turning into a new leg
HEAD_LP    = 1 - 0.7^(FRAME_DT/0.2);   % heading low-pass (0.3 per 0.2 s)
OFF_LP     = 1 - 0.5^(FRAME_DT/0.2);   % low-pass of the sideways offset from the line (0.5 per 0.2 s)
HEAD_MAX   = 15*pi/180*FRAME_DT;   % max heading change, 15 deg/s (bends > 20 deg are corners anyway)
YAW_STEP   = 60*pi/180*FRAME_DT;   % max yaw lead, 60 deg/s
VERTEX_TOL = 0.04;         % corner vertex reached
CORNER_T   = 6.0;          % s, give up refining the vertex / turning
TURN_TOL   = 5*pi/180;     % leave a corner only once the nose is this close to the new leg
COAST_STEP = 0.12;         % carrot while coasting past the line end
COAST_MAX  = 0.80;         % then search
SEARCH_R   = 0.60;         % spiral radius limit
SEARCH_T   = 25;           % s, then land where we are
LAND_TOL   = 0.05;         % centred over the circle
LAND_HOLD  = round(1.0/FRAME_DT);   % centred for 1 s before descending
DESC_RATE  = 0.35;         % m/s down to 0.8 m, then DESC_SLOW
DESC_SLOW  = 0.20;         % m/s for the last 0.8 m (gentle touchdown)
CLIMB_RATE = 0.5;          % m/s, take-off height target ramp
TAKEOFF_TOL= 0.10;         % m, at altitude when within this for 1 s
CIRC_FREEZE= 0.45;         % below this height the circle fills the view: freeze the estimate
LAND_Z     = 0.10;         % final height command, m below the floor (gentle touchdown, no deep wind-up)
TOUCH_H    = 0.06;         % height counted as landed
NPATH      = 3000;         % flown path points kept (1 cm spacing)
SCENE_OFF  = [0.5 1];      % the 3D scene draws the drone at NED + this
PATCH_E0   = 3.1; PATCH_DE = 0.1;   % red brick patch at the arena corner (scene coords)
HARDWARE   = true;         % <<< set true before deploying to the real drone (no brick patch on a real floor)
PATCH_N    = [4.17 3.73 3.59 3.58 3.64 3.70 3.72 3.77 3.87 3.98 4.11 4.20 4.30];
if HARDWARE, PATCH_N = 99 * ones(1, 13); end   % 99 m: outside any arena, so inPatch is never true

%% ---------------- memory ----------------
persistent st t tState cnt home sp heading nextDir vertex pathXY arc nP lastLineP coast0
persistent searchC searchK circLast circHave circCnt circEst spD hist lineOff prevP
persistent leadPrev yOut yRate zOut vzOut
if isempty(st)
    st = 0; t = 0; tState = 0; cnt = 0;
    home = [double(pos(1)) double(pos(2)) double(pos(4))];
    sp = [home(1) home(2) 0 home(3)];
    heading = NaN; nextDir = NaN; vertex = [NaN NaN];
    pathXY = zeros(NPATH, 2); arc = zeros(NPATH, 1); nP = 0;
    lastLineP = home(1:2); coast0 = home(1:2); searchC = home(1:2); searchK = 0;
    circLast = [0 0]; circHave = false; circCnt = 0; circEst = home(1:2); spD = 0;
    hist = repmat(double(pos(:)).', LAG_POS + 1, 1);
    lineOff = NaN; prevP = home(1:2);
    leadPrev = [0 0]; yOut = home(3); yRate = 0; zOut = 0; vzOut = 0;
end
hist = [double(pos(:)).'; hist(1:end-1, :)];
p    = hist(1, 1:2);                       % now
yaw  = hist(1, 4);
pImg = hist(LAG_POS + 1, 1:2);             % where the camera was when the image was taken
yImg = hist(LAG_YAW + 1, 4);
h    = max(-hist(1, 3), 0.02);
hImg = max(-hist(LAG_POS + 1, 3), 0.02);
mpp  = K08 * hImg / 0.8;

%% ---------------- 1. pixels on the floor, brick patch removed ----------------
[H, W] = size(mask);
gN = zeros(H, W); gE = zeros(H, W); red = false(H, W);
cy = cos(yImg); sy = sin(yImg);
for c = 1:W
    for r = 1:H
        f = (RC - r) * mpp; q = (c - CC) * mpp;
        gN(r,c) = pImg(1) + f*cy - q*sy;
        gE(r,c) = pImg(2) + f*sy + q*cy;
        if mask(r,c)
            red(r,c) = ~inPatch(gN(r,c) + SCENE_OFF(1), gE(r,c) + SCENE_OFF(2), PATCH_E0, PATCH_DE, PATCH_N);
        end
    end
end

%% ---------------- 2. blobs: line pixels vs circle ----------------
[lab, nLab] = bwlabel(red, 8);
nL = min(nLab, 64);
sN = zeros(nL,1); sE = sN; sNN = sN; sEE = sN; sNE = sN; n = sN; brd = false(nL,1);
for c = 1:W
    for r = 1:H
        k = lab(r,c);
        if k > 0 && k <= nL
            n(k) = n(k) + 1; sN(k) = sN(k) + gN(r,c); sE(k) = sE(k) + gE(r,c);
            sNN(k) = sNN(k) + gN(r,c)^2; sEE(k) = sEE(k) + gE(r,c)^2; sNE(k) = sNE(k) + gN(r,c)*gE(r,c);
            if r <= 2 || r >= H-1 || c <= 2 || c >= W-1, brd(k) = true; end
        end
    end
end
isLine = false(nL,1); circNow = [0 0]; circSeen = false;
for k = 1:nL
    if n(k) < RED_MIN_PX, continue; end
    mN = sN(k)/n(k); mE = sE(k)/n(k);
    a = sNN(k)/n(k) - mN^2; b = sEE(k)/n(k) - mE^2; cxy = sNE(k)/n(k) - mN*mE;
    tr = (a + b)/2; dd = sqrt(max(((a - b)/2)^2 + cxy^2, 0));
    l1 = max(tr + dd, 0); l2 = max(tr - dd, 0);
    len = sqrt(12*l1); area = n(k) * mpp^2; areaC = pi*(CIRC_D/2)^2;
    if ~brd(k) && abs(area/areaC - 1) < CIRC_TOL && sqrt(l2/max(l1, 1e-12)) > CIRC_ROUND
        circNow = [mN mE]; circSeen = true;
    elseif len > LINE_LEN || (brd(k) && len > 0.22)
        isLine(k) = true;
    end
end
lineM = false(H, W); nLinePx = 0;
for c = 1:W
    for r = 1:H
        k = lab(r,c);
        if k > 0 && k <= nL && isLine(k), lineM(r,c) = true; nLinePx = nLinePx + 1; end
    end
end
% circle confirmation (estimate moved to the current frame: it is a floor point)
if circSeen && circHave && norm(circNow - circLast) < CIRC_JUMP
    circCnt = circCnt + 1;
elseif circSeen
    circCnt = 1;
else
    circCnt = 0;
end
circHave = circSeen; if circSeen, circLast = circNow; end
circOK = circCnt >= CIRC_FR;

%% ---------------- 3. ring scan around the image-time position ----------------
tot = zeros(NB,1); hit = zeros(NB,1); sumN = zeros(NB,1); sumE = zeros(NB,1);
for c = 1:W
    for r = 1:H
        dN = gN(r,c) - pImg(1); dE = gE(r,c) - pImg(2);
        if abs(sqrt(dN^2 + dE^2) - LOOK_R) < RING_HALF
            bb = min(floor((atan2(dE, dN) + pi) / (2*pi) * NB) + 1, NB);
            tot(bb) = tot(bb) + 1;
            if lineM(r,c), hit(bb) = hit(bb) + 1; sumN(bb) = sumN(bb) + gN(r,c); sumE(bb) = sumE(bb) + gE(r,c); end
        end
    end
end
occ = hit > OCC_FRAC * tot & tot >= 4;
MAXC = 12; cand = zeros(MAXC, 4); nc = 0;          % [bearing N E widthBins], bearing from the drone now
if any(occ) && ~all(occ)
    s0 = 1;
    for j = NB:-1:1, if ~occ(j), s0 = j; end, end   % first empty bin (scalar, codegen-safe)
    runN = 0; rN = 0; rE = 0; rH = 0;
    for j = 0:NB
        bb = mod(s0 - 1 + j, NB) + 1;
        if occ(bb) && j < NB
            runN = runN + 1; rN = rN + sumN(bb); rE = rE + sumE(bb); rH = rH + hit(bb);
        elseif runN > 0
            if nc < MAXC
                nc = nc + 1; qN = rN/rH; qE = rE/rH;
                cand(nc,:) = [atan2(qE - p(2), qN - p(1)) qN qE runN];
            end
            runN = 0; rN = 0; rE = 0; rH = 0;
        end
    end
end

%% ---------------- 4. flown path ----------------
if nP == 0
    nP = 1; pathXY(1,:) = p; arc(1) = 0;
elseif norm(p - pathXY(nP,:)) > 0.01
    if nP == NPATH                                   % full: drop the oldest half
        hN = NPATH/2; pathXY(1:hN,:) = pathXY(hN+1:end,:); arc(1:hN) = arc(hN+1:end); nP = hN;
    end
    nP = nP + 1; arc(nP) = arc(nP-1) + norm(p - pathXY(nP-1,:)); pathXY(nP,:) = p;
end

%% ---------------- 5. filter the crossings ----------------
if nc > 0 && st ~= 0
    keep = true(nc,1);
    if arc(nP) > 0.08
        kc = 1;
        for i = nP:-1:1, if arc(i) <= arc(nP) - CRUMB_ARC, kc = i; break; end, end
        back = atan2(pathXY(kc,2) - p(2), pathXY(kc,1) - p(1));
        for i = 1:nc, if abs(wrapA(cand(i,1) - back)) < BACK_ZONE, keep(i) = false; end, end
    end
    for i = 1:nc
        for j = 1:nP
            if (pathXY(j,1) - cand(i,2))^2 + (pathXY(j,2) - cand(i,3))^2 < OLD_TOL^2, keep(i) = false; break; end
        end
    end
    [cand, nc] = compact(cand, nc, keep);
    keep = true(nc,1);
    for i = 1:nc, keep(i) = redFrac(p, cand(i,2:3), lineM, pImg, yImg, mpp, RC, CC) >= JOIN_FRAC; end
    if any(keep), [cand, nc] = compact(cand, nc, keep); end
end
if nc > 0, lastLineP = p; end

%% ---------------- 6. state machine ----------------
spN = sp(1); spE = sp(2); zC = -ALT; yC = sp(4);
switch st
    case 0  % TAKEOFF
        % ramp the height target (a 2 m step winds up PX4's height integrator: +0.3 m overshoot)
        spD = max(spD - CLIMB_RATE*FRAME_DT, -ALT);
        spN = home(1); spE = home(2); yC = home(3); zC = spD;
        if abs(h - ALT) < TAKEOFF_TOL, cnt = cnt + 1; else, cnt = 0; end
        if cnt >= round(1.0/FRAME_DT), st = 1; cnt = 0; tState = t; end
    case 1  % FOLLOW
        if isnan(heading)
            if nc == 0
                if t - tState > 1.0 && norm(p - home(1:2)) < 0.6
                    spN = p(1) + 0.04*cos(yaw); spE = p(2) + 0.04*sin(yaw);
                end
            else
                bi = 1; bd = inf;
                for i = 1:nc, d1 = abs(wrapA(cand(i,1) - yaw)); if d1 < bd, bd = d1; bi = i; end, end
                heading = cand(bi,1);
            end
        end
        if ~isnan(heading)
            u = [cos(heading) sin(heading)];
            [v, dAhead] = findVertex(p, heading, lineM, gN, gE);
            cone = AHEAD; if nc > 1, cone = AHEAD_STR; end
            ai = 0; ad = inf; mi = 0; md = inf;
            for i = 1:nc
                d1 = abs(wrapA(cand(i,1) - heading));
                if d1 < cone && d1 < ad, ad = d1; ai = i; end
                if d1 < md, md = d1; mi = i; end
            end
            vNow = norm(p - prevP) / FRAME_DT;
            dStop = vNow*LAG_T + vNow^2/(2*A_BRAKE);
            if ~isnan(v(1)) && dot(v - p, u) < max(LOOK_R + 0.03, dStop + 0.05) && t - tState > 0.6
                vertex = v; nextDir = NaN; st = 2; tState = t; lineOff = NaN;
                spN = v(1); spE = v(2); yC = yawToward(yaw, heading, YAW_STEP);
            elseif ai > 0
                if cand(ai,4) > FAT_BINS
                    dirC = heading;
                else
                    dirC = cand(ai,1);
                    [lineDir, lc] = fitLine(pImg, heading, lineM, gN, gE);
                    if ~isnan(lineDir)
                        heading = heading + max(-HEAD_MAX, min(HEAD_MAX, HEAD_LP*wrapA(lineDir - heading)));
                        % signed offset of the line from us, smoothed over frames
                        off = -(p(1) - lc(1))*sin(heading) + (p(2) - lc(2))*cos(heading);
                        if isnan(lineOff), lineOff = off; else, lineOff = lineOff + OFF_LP*(off - lineOff); end
                    end
                end
                if cand(ai,4) <= FAT_BINS && ~isnan(lineOff)
                    % setpoint ON the line: our foot-point on it + STEP along it. PX4 position loop does the rest.
                    % speed: as fast as we can still stop before the end of the line we can see
                    dGo = max(dAhead - END_MARGIN, 0);
                    vOk = A_BRAKE * (-LAG_T + sqrt(LAG_T^2 + 2*dGo/A_BRAKE));
                    k2 = max(cos(min(abs(wrapA(heading - sp(4))), pi/2)), 0)^2;   % slow while the nose still turns
                    stp = max(STEP_MIN, min(V_MAX, vOk) * T_CARROT * k2);
                    spN = p(1) + stp*cos(heading) + lineOff*sin(heading);
                    spE = p(2) + stp*sin(heading) - lineOff*cos(heading);
                else
                    k2 = max(cos(min(abs(wrapA(dirC - yaw)), pi/2)), 0)^2;
                    stp = STEP_MIN + (STEP_MAX - STEP_MIN)*k2;
                    spN = p(1) + stp*cos(dirC); spE = p(2) + stp*sin(dirC);
                end
                yC = yawToward(sp(4), heading, YAW_STEP);   % nose follows the smoothed line direction
            elseif isnan(v(1)) && redFrac(p, p + 0.2*u, lineM, pImg, yImg, mpp, RC, CC) > 0.6
                spN = p(1) + COAST_STEP*u(1); spE = p(2) + COAST_STEP*u(2); yC = yawToward(yaw, heading, YAW_STEP);
            elseif mi > 0
                nextDir = cand(mi,1); vertex = v; if isnan(v(1)), vertex = p; end
                st = 2; tState = t; lineOff = NaN; spN = vertex(1); spE = vertex(2); yC = yawToward(yaw, heading, YAW_STEP);
            else
                st = 3; coast0 = p; tState = t;
                spN = p(1) + COAST_STEP*u(1); spE = p(2) + COAST_STEP*u(2); yC = yawToward(yaw, heading, YAW_STEP);
            end
        end
    case 2  % CORNER
        v = findVertex(p, heading, lineM, gN, gE);
        if ~isnan(v(1))
            dv = v - vertex; aw = dv(1)*cos(heading) + dv(2)*sin(heading); lat = -dv(1)*sin(heading) + dv(2)*cos(heading);
            if norm(dv) < 0.10
                vertex = 0.5*vertex + 0.5*v;
            elseif aw > 0.05 && aw < 0.50 && abs(lat) < 0.06
                vertex = v;          % the line goes on further than first seen (tip was at the image edge): move on
            end
        end
        % brake along a profile v = sqrt(2 a d) instead of jumping the setpoint onto the vertex
        dV = norm(vertex - p);
        if dV > 0.04
            uV = (vertex - p) / dV;
            vAl = dot(p - prevP, uV) / FRAME_DT;               % speed towards the vertex
            vProf = sqrt(2*A_BRAKE*max(dV - 0.03, 0));          % allowed speed at this distance
            if vAl > vProf
                % PX4 alone brakes at only KV_XY*(speed error) ~0.6 m/s^2: ask for a lower speed so the
                % resulting deceleration is what is needed to stop at the vertex
                aNeed = min(A_MAX, vAl^2 / (2*max(dV - 0.03, 0.02)));
                vCmd = max(vAl - aNeed/KV_XY, -0.3);
            else
                vCmd = vProf;
            end
            stp = vCmd / KP_XY;                                   % may be behind us (negative)
            stp = min(stp, dV);
            spN = p(1) + stp*uV(1); spE = p(2) + stp*uV(2);
        else
            spN = vertex(1); spE = vertex(2);
        end
        if nc > 0
            bi = 1; bd = inf;
            for i = 1:nc, d1 = abs(wrapA(cand(i,1) - heading)); if d1 < bd, bd = d1; bi = i; end, end
            nextDir = cand(bi,1);
        end
        if ~isnan(nextDir), yC = yawToward(sp(4), nextDir, YAW_STEP); else, yC = sp(4); end   % smooth pivot
        aligned = isnan(nextDir) || abs(wrapA(yaw - nextDir)) < TURN_TOL;   % pivot first, then go
        if (norm(p - vertex) < VERTEX_TOL && aligned) || t - tState > CORNER_T
            if isnan(nextDir), st = 3; coast0 = p; else, heading = nextDir; st = 1; end
            tState = t;
        end
    case 3  % COAST
        minD = inf; for i = 1:nc, minD = min(minD, abs(wrapA(cand(i,1) - heading))); end
        if circOK
            st = 5; circEst = circLast; cnt = 0;
        elseif minD < AHEAD
            st = 1; tState = t;
        elseif norm(p - coast0) > COAST_MAX
            st = 4; tState = t; searchC = lastLineP; searchK = 0;
        else
            spN = p(1) + COAST_STEP*cos(heading); spE = p(2) + COAST_STEP*sin(heading); yC = yawToward(yaw, heading, YAW_STEP);
        end
    case 4  % SEARCH
        if circOK
            st = 5; circEst = circLast; cnt = 0;
        elseif nc > 0
            bi = 1; bd = inf;
            for i = 1:nc, d1 = abs(wrapA(cand(i,1) - heading)); if d1 < bd, bd = d1; bi = i; end, end
            heading = cand(bi,1); st = 1; tState = t;
        elseif t - tState > SEARCH_T
            st = 6; circEst = p; spD = -h;
        else
            q = spiralPt(searchC, searchK, SEARCH_R);
            if norm(p - q) < 0.04, searchK = searchK + 1; q = spiralPt(searchC, searchK, SEARCH_R); end
            spN = q(1); spE = q(2); yC = yaw;
        end
    case 5  % APPROACH
        if circSeen && norm(circNow - circEst) < 0.08, circEst = 0.7*circEst + 0.3*circNow; end
        spN = circEst(1); spE = circEst(2); yC = yaw;
        if norm(p - circEst) < LAND_TOL, cnt = cnt + 1; else, cnt = 0; end
        if cnt >= LAND_HOLD, st = 6; spD = -h; end   % descend from where we are
    case 6  % DESCEND
        if h > CIRC_FREEZE && circSeen && norm(circNow - circEst) < 0.05, circEst = 0.8*circEst + 0.2*circNow; end
        if h > 0.8, rate = DESC_RATE; else, rate = DESC_SLOW; end
        spD = min(spD + rate*FRAME_DT, LAND_Z);
        spN = circEst(1); spE = circEst(2); zC = spD; yC = yaw;
        if h < TOUCH_H, st = 7; end
    otherwise  % LANDED: hold on the floor
        spN = sp(1); spE = sp(2); zC = LAND_Z; yC = sp(4);
end
%% ---------------- output shaper (all modes) ----------------
% horizontal: PX4 flies at about KP_XY * (setpoint - position), so limit how fast that lead changes
lead = [spN - p(1), spE - p(2)];
dl = lead - leadPrev;
if norm(lead) > norm(leadPrev), aLim = A_UP; else, aLim = A_DOWN; end
mx = aLim * FRAME_DT / KP_XY;
if norm(dl) > mx, lead = leadPrev + dl * (mx / norm(dl)); end
leadPrev = lead;
spN = p(1) + lead(1); spE = p(2) + lead(2);
% yaw: rate ramps up, cruises, and ramps down to stop exactly on the target heading
eY = wrapA(yC - yOut);
rDes = sign(eY) * min([YAW_RATE, sqrt(2*YAW_ACC*abs(eY)), abs(eY)/FRAME_DT]);
yRate = yRate + max(-YAW_ACC*FRAME_DT, min(YAW_ACC*FRAME_DT, rDes - yRate));
yOut = wrapA(yOut + yRate*FRAME_DT);
yC = yOut;
% height: same ramp-up / ramp-down for take-off, level-off, descent and touchdown
eZ = zC - zOut;
vzDes = sign(eZ) * min([VZ_MAX, sqrt(2*AZ_MAX*abs(eZ)), abs(eZ)/FRAME_DT]);
vzOut = vzOut + max(-AZ_MAX*FRAME_DT, min(AZ_MAX*FRAME_DT, vzDes - vzOut));
zOut = zOut + vzOut*FRAME_DT;
zC = zOut;
sp = [spN spE zC yC];
t = t + FRAME_DT;
prevP = p;

% localToNED rotates [x_b y_b] by the commanded yaw, so pre-rotate the target back
traj = [ spN*cos(yC) + spE*sin(yC);
        -spN*sin(yC) + spE*cos(yC);
         zC;
         yC];
hd = heading; if isnan(hd), hd = 99; end
dbg = [st; yC; nLinePx; hd];
end

%% =============================== helpers ===============================
function [c2, n2] = compact(c, n, keep)
c2 = zeros(size(c)); n2 = 0;
for i = 1:n
    if keep(i), n2 = n2 + 1; c2(n2,:) = c(i,:); end
end
end

function f = redFrac(a, b, lineM, pImg, yImg, mpp, RC, CC)
% Fraction of the floor segment a->b (first 6 cm skipped) that lies on line pixels.
[H, W] = size(lineM); ns = 12; hitN = 0; L = max(norm(b - a), 0.06);
cy = cos(yImg); sy = sin(yImg);
for i = 1:ns
    s = 0.06/L + (1 - 0.06/L) * (i - 1)/(ns - 1);
    wN = a(1) + s*(b(1) - a(1)) - pImg(1); wE = a(2) + s*(b(2) - a(2)) - pImg(2);
    fwd = wN*cy + wE*sy; rgt = -wN*sy + wE*cy;
    r = round(RC - fwd/mpp); c = round(CC + rgt/mpp);
    if r >= 1 && r <= H && c >= 1 && c <= W && lineM(r,c), hitN = hitN + 1; end
end
f = hitN / ns;
end

function [v, dAvail] = findVertex(p, heading, lineM, gN, gE)
% End of our straight line ahead (farthest line pixel near the track axis), NaN if it runs out of view.
[H, W] = size(lineM); u1 = cos(heading); u2 = sin(heading);
ns = 0; sMaxB = inf; sv = zeros(1, H*W); lsum = 0;
for c = 1:W
    for r = 1:H
        s = (gN(r,c) - p(1))*u1 + (gE(r,c) - p(2))*u2;
        l = -(gN(r,c) - p(1))*u2 + (gE(r,c) - p(2))*u1;
        if abs(l) < 0.08 && s > -0.1
            if r <= 3 || r >= H-2 || c <= 3 || c >= W-2
                if s > 0, sMaxB = min(sMaxB, s); end
            end
            if lineM(r,c), ns = ns + 1; sv(ns) = s; lsum = lsum + l; end
        end
    end
end
v = [NaN NaN]; dAvail = 0;
if ns < 10, return; end
ss = sort(sv(1:ns));
sEnd = ss(max(1, round(0.98*ns)));
dAvail = min(sEnd, sMaxB);                     % line we can see ahead (to its end or the image edge)
if sEnd > sMaxB - 0.08, return; end   % need 8 cm of clear floor past the end (edge rows fade)
l0 = lsum / ns;
v = [p(1) + (sEnd - 0.05)*u1 - l0*u2, p(2) + (sEnd - 0.05)*u2 + l0*u1];
end

function [d, ctr] = fitLine(c0, heading, lineM, gN, gE)
% Direction of our line: principal axis of line pixels 0.05-0.30 m ahead of c0 and within 0.08 m of the
% current track axis (a crossing line is mostly outside that band). NaN if too few pixels.
[H, W] = size(lineM); u1 = cos(heading); u2 = sin(heading);
n = 0; sN = 0; sE = 0; sNN = 0; sEE = 0; sNE = 0;
for c = 1:W
    for r = 1:H
        if lineM(r,c)
            dN = gN(r,c) - c0(1); dE = gE(r,c) - c0(2);
            s = dN*u1 + dE*u2; l = -dN*u2 + dE*u1;
            if s > 0.04 && s < 0.28 && abs(l) < 0.08   % ahead only: the corner blob / old leg behind would tilt the fit
                n = n + 1; sN = sN + dN; sE = sE + dE; sNN = sNN + dN^2; sEE = sEE + dE^2; sNE = sNE + dN*dE;
            end
        end
    end
end
d = NaN; ctr = c0;
if n < 150, return; end
ctr = [c0(1) + sN/n, c0(2) + sE/n];            % a point on the line (floor coordinates)
a = sNN/n - (sN/n)^2; b = sEE/n - (sE/n)^2; cxy = sNE/n - (sN/n)*(sE/n);
d = 0.5*atan2(2*cxy, a - b);                    % principal axis angle (bearing, N = 0)
if cos(d - heading) < 0, d = d + pi; end         % point it the way we travel
d = wrapA(d);
end

function y = yawToward(yaw, dirC, stepMax)
y = yaw + max(-stepMax, min(stepMax, wrapA(dirC - yaw)));
end

function q = spiralPt(c, k, rMax)
r = min(0.08 + 0.03*k, rMax); a = k*pi/3;
q = [c(1) + r*cos(a), c(2) + r*sin(a)];
end

function a = wrapA(a)
a = mod(a + pi, 2*pi) - pi;
end

function tf = inPatch(north, east, E0, DE, PN)
% True if scene point (north, east) lies on the red brick patch (boundary table from the team model).
k = (east - E0) / DE + 1;
k0 = min(max(floor(k), 1), numel(PN) - 1);
t  = min(max(k - k0, 0), 1);
tf = (k >= 0.5) && (k <= numel(PN) + 0.5) && (north > PN(k0) * (1 - t) + PN(k0 + 1) * t);
end
