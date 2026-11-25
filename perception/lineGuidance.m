function [traj, dbg] = lineGuidance(mask, pos)
%#codegen
% Mask red pixels off the brick patch, measure where the line crosses
% a look-ahead ring, pick the crossing that continues the line (not
% the one we came along), steer a carrot point toward it, and run the
% state machine TAKEOFF, FOLLOW, COAST, SETTLE, LAND.
% mask: 120x160 logical, true = red. Image up = drone forward, image right = drone right.
% pos:  [x y z yaw] of the drone (NED-style, z up is negative, yaw clockwise).
% traj: [x_b; y_b; z; yaw] in the form localToNED expects (x/y rotated by commanded yaw).
% dbg:  [state; heading; centrePixels; dirW or 99] for logging.
% A landing circle clipped by a wall is re-centred from its uncut extent.
% EDITING: the copy that flies is embedded in perception/PerceptionAndGuidance.slx. After editing this file,
% run harness/adopt_guidance.m to write it into the model, then re-run the tests (see harness/README.md).
% x69 (2026-11-25, Round 2 hardware): HARDWARE switch turns off the two sim-scene fixes (brick-patch
% red filter, Track Builder spawn offset in the start search); landing ends 0.3 m below the floor, not 1 m.

% Target: true = real drone (Round 2 arena), false = Round 1 Simulink/Unreal scene
HARDWARE = true;

% Camera and look-ahead ring
K    = 0.0042;      % metres per pixel at hover height (line 0.1 m = 24 px, measured)
RC   = 60.5;        % image centre row
CC   = 80.5;        % image centre column
R1   = 48;          % look-ahead ring inner radius, px
R2   = 58;          % look-ahead ring outer radius, px (image half-height is 60)
RIN  = 35;          % inner disk radius used to centre on the landing circle, px
NB   = 72;          % ring angle bins (5 deg each)

% Flight
HOVER_Z   = -0.8;   % flight altitude (0.8 m up)
FRAME_DT  = 0.2;    % camera frame time interval, s

% Line following and yaw
% Carrot distance while the nose still points away from the line (corner: nearly stop and turn), m
STEP      = 0.06;
STEP_MAX  = 0.22;   % carrot distance on a straight, m (PX4 flies faster to a farther target)
OCC_FRAC  = 0.3;    % ring bin occupancy fraction threshold
AHEAD_CONE = 60*pi/180; % cone angle for ahead red detection
YAW_GAIN  = 0.3;    % fraction of the heading error corrected per frame (fast turn at a corner while slowed down)
YAW_FINE  = 0.12;   % gain for small heading errors (damps sway on straights and the overshoot tail after a turn)
YAW_SMALL = 15*pi/180;   % heading errors below this use YAW_FINE
YAW_MAX   = 10*pi/180;   % largest heading change per frame
LAG_N     = 3;           % frames of position lag used to place the ring crossing (x61)

% Corners and V tips
BACK_ZONE = 25*pi/180;   % a crossing within 25 deg of the breadcrumb direction is the line we came along
LOCK_TURN = 60*pi/180;   % a new direction this far from the travel direction is a corner turn ...
LOCK_D    = 0.2;          % ... held for this distance so a one-frame dropout at the vertex cannot undo it, m
% During a committed turn, a frame-to-frame direction jump bigger than this is a dropout, not the line
LOCK_TOL  = 25*pi/180;
% A ring crossing closer than this to a point of the path flown before the last sharp corner is that old leg, m
OLD_TOL   = 0.09;
OLD_DMIN  = 0.25;   % old-leg filter starts this far past the corner (the turn itself is never filtered), m
OLD_DMAX  = 1.0;    % ... and ends this far past it, m
% Largest red count under the drone in the last 2 frames above this when the line ended =
% two legs overlapping (a single line is 1600-1650)
FAT_N     = 1750;
U_HOLD    = 1.2;          % after a U-turn, prefer the diverging leg for this far, m
U_WIDE    = 45*pi/180;    % ... among crossings within this angle of the incoming line's direction

% Track end and landing circle
% Circle: narrowest spread at least this fraction of the widest (line ~0.35, line-end stub <= 0.64, circle ~0.9)
CIRCLE_ROUND = 0.75;
CIRCLE_MAJOR = 14;   % circle: widest width (std, px) below this; a line or corner is ~18-20
CIRCLE_MIN_N = 900;  % min red count for circle detection under us
BLOB_N    = 600;  % circle candidate: more red pixels than this under the drone, and not line-long
% ... and at least this many more than the lowest count since the line ended (a line-end stub only shrinks)
BLOB_RISE = 300;
COAST_MAX = 0.8;    % give up looking for the circle after coasting this far, m
COAST_STEP = 0.12;   % carrot distance while coasting toward the circle, m (slower approach, less overshoot)
COAST_AVG = 12;     % frames of line direction averaged for the coast heading (x66)
% Frames in a row the circle test must pass before settling (a V tip lasts 1-2 frames, the real circle ~10)
CIRC_FR   = 4;
% SETTLE may be cancelled during its first 10 frames (2 s) if a line shows up and no round circle is under us
ABORT_FR  = 10;
UTURN_D   = 0.1;  % coast at least this far past a dead end before taking a leg that folds back beside ours, m
SLOW_FRAMES = 10;   % after the line ends once (corner or track end), stay slow this many frames

% Landing
LAND_RATE = 0.03;   % descent per frame while landing, m (x5 per second)
% Final height command while landing, m below floor: below the floor, so thrust drops and the drone cannot
% slide in wind. x68 used 1.0 m; 0.3 m bounds the height error (integrator wind-up after touchdown) and still
% covers ~0.3 m of barometer drift in the real drone's height estimate.
LAND_FLOOR = 0.3;
LAND_KI    = 0.1;   % integral gain per frame: slowly cancels a steady wind push while centring
SETTLE_TOL = 0.010; % centred when the circle is within this distance of the drone, m
% Landing: forward trim of the circle offset; x53 stopped 9.3 mm short along its heading over 91 landings
% and the loop amplifies this trim about 2.7x (measured on default and w31), m
CAM_FWD    = 0.0035;
SETTLE_MAX = 40;    % give up waiting to be centred after this many frames (8 s)
STILL_V    = 0.05;  % drone counts as still below this ground speed, m/s (camera frames are 0.2 s apart)
LAND_MINH  = 0.45; % stop camera corrections below this height, m (circle fills the view)
LAND_MIN_N = 20;    % min red count for landing corrections
SETTLE_MIN = 8;     % min hold frames before settling is done
CIRC_R   = 0.097;  % visible radius of the landing circle in the mask at hover height, m (measured on eastwall)
CLIP_TOL = 0.02;   % blob shorter than 2*CIRC_R by more than this along N or E = clipped (wall hides part), m

% Start search
START_MIN_N = 200;  % min red count at start
START_DELAY = 5;    % frames before start search kicks in
SRCH_PITCH = 0.30;   % m of radius gained per spiral turn
SRCH_RMAX  = 2.0;    % max spiral radius, m
SRCH_STEP  = 0.06;   % carrot step for start search

% Breadcrumb
CRUMB     = 0.25;   % breadcrumb trails this far behind the drone, m
CRUMB_SEG = 0.02;   % record a path point each time the drone has moved this far, m
CRUMB_MIN_D = 0.1;  % min distance for breadcrumb direction
NPATH     = 30;     % path points kept (0.6 m of flown path)

% Brick patch
% The 3D scene draws the drone at NED + this, m. On the real drone the takeoff point is the origin.
if HARDWARE
    SCENE_OFF = [0 0];
else
    SCENE_OFF = [0.5 1];
end
PATCH_E0  = 3.1;      % red background patch at the arena corner: first East sample, m (scene)
PATCH_DE  = 0.1;      % spacing of the samples, m
% Patch starts at this North (scene) for each East sample; E 3.5-4.3 re-measured on v31 frames (x49)
PATCH_N   = [4.17 3.73 3.59 3.58 3.64 3.70 3.72 3.77 ...
           3.87 3.98 4.11 4.20 4.30];

% State machine constants
S_TAKEOFF = 0;
S_FOLLOW  = 1;
S_COAST   = 2;
S_SETTLE  = 3;
S_LAND    = 4;

% State machine: state holdCnt
persistent state holdCnt
% Commands sent to the flight controller: hCmd zCmd tgt
persistent hCmd zCmd tgt
% Line following: first travel psiImg slowCnt prevXY
persistent first travel psiImg slowCnt prevXY
% Position history for the image-time carrot (row 1 is now, row LAG_N+1 is image time)
persistent xyHist
% Breadcrumb path: crumb pathXY nPath
persistent crumb pathXY nPath
% Corner lock: lockOn lockFrom
persistent lockOn lockFrom
% U-turn memory: uturnOn uturnFrom uturnDir
persistent uturnOn uturnFrom uturnDir
% Old-leg memory: oldOn oldCorner oldPath oldLen
persistent oldOn oldCorner oldPath oldLen
% Track end and landing circle: coastFrom coastN nMin circCnt nHist
persistent coastFrom coastN nMin circCnt nHist
% Coast heading: dirHist nDir
persistent dirHist nDir
% Start search: startCnt startXY srchOn srchAng
persistent startCnt startXY srchOn srchAng
% Landing: landI centred clipBias
persistent landI centred clipBias

if isempty(state)
    state = S_TAKEOFF;
    hCmd = 0;
    tgt = [0 0];
    zCmd = HOVER_Z;
    holdCnt = 0;
    first = true;
    travel = 0;
    crumb = [0 0];
    coastFrom = [0 0];
    psiImg = 0;
    slowCnt = 0;
    landI = [0 0];
    prevXY = [0 0];
    xyHist = [double(pos(1)) double(pos(2)); double(pos(1)) double(pos(2)); ...
              double(pos(1)) double(pos(2)); double(pos(1)) double(pos(2))];
    centred = false;
    clipBias = [0 0];
    nMin = inf;
    uturnOn = false;
    uturnFrom = [0 0];
    uturnDir = 0;
    lockOn = false;
    lockFrom = [0 0];
    oldOn = false;
    oldCorner = [0 0];
    oldPath = zeros(NPATH, 2);
    oldLen = 0;
    nHist = zeros(1, 5);
    coastN = 0;
    circCnt = 0;
    dirHist = zeros(COAST_AVG, 1);
    nDir = 0;
    startCnt = 0;
    startXY = [0 0];
    srchOn = false;
    srchAng = 0;
    pathXY = zeros(NPATH, 2);
    nPath = 0;
end

cam.K = K;
cam.HOVER_Z = HOVER_Z;
cam.RC = RC;
cam.CC = CC;
cam.R1 = R1;
cam.R2 = R2;
cam.RIN = RIN;
cam.NB = NB;
cam.CIRC_R = CIRC_R;
cam.CLIP_TOL = CLIP_TOL;
scene.OFF = SCENE_OFF;
scene.PATCH_E0 = PATCH_E0;
scene.PATCH_DE = PATCH_DE;
scene.PATCH_N = PATCH_N;
scene.USE_PATCH = ~HARDWARE;   % the red brick patch exists only in the sim scene

x = double(pos(1));
y = double(pos(2));
z = double(pos(3));
psiNow = double(pos(4));
psi = psiImg;
psiImg = psiNow;   % image is one frame old: use the yaw it was taken at
% Moved less than STILL_V*FRAME_DT m since last frame
still = hypot(x - prevXY(1), y - prevXY(2)) / FRAME_DT < STILL_V;
prevXY = [x y];
% Shift the position history: row 1 now, row 2 one frame ago, and so on
xyHist = [x y; xyHist(1:3,:)];

[pathXY, nPath, crumb, behind, useBehind] = updateBreadcrumb(...
    x, y, pathXY, nPath, NPATH, CRUMB, CRUMB_SEG, CRUMB_MIN_D);

[occ, hit, tot, hitAll, centre, metresPerPx] = scanImage(...
    mask, x, y, psi, z, cam, scene, OCC_FRAC);
nCentre = centre.n;

% Any red ahead at all (e.g. the landing circle, not connected to the line end under us)
aheadRed = false;
for b = 1:NB
    wb = wrapAngle(psi - pi + (b - 0.5) * 2*pi/NB);
    if hitAll(b) > OCC_FRAC * tot(b) && abs(wrapAngle(wb - travel)) < AHEAD_CONE, aheadRed = true; end
end

[centreDir, minorStd, majorStd] = centreBlobShape(centre, psi);

% Round red blob under us (works when shading shrinks it)
isCircle = nCentre > CIRCLE_MIN_N && majorStd < CIRCLE_MAJOR && ...
    minorStd > CIRCLE_ROUND * majorStd;
% Lowest red count since a line was last under us
if nCentre > BLOB_N && majorStd >= CIRCLE_MAJOR
    nMin = inf;
else
    nMin = min(nMin, nCentre);
end
% Blob growing after the gap: the circle, even if partly hidden
isBlob = nCentre > max(BLOB_N, nMin + BLOB_RISE) && ...
    majorStd < CIRCLE_MAJOR;

% ---- split occupied bins into runs (line crossings of the ring), pick one ----
% First lock-on: the line starts under the drone, so match the direction of the red near the centre
if first, ref = centreDir; else, ref = travel; end
if uturnOn && hypot(x - uturnFrom(1), y - uturnFrom(2)) > U_HOLD, uturnOn = false; end
Rm = metresPerPx * (R1 + R2) / 2;   % ring radius in metres at the current height
oldUse = oldOn && state == S_FOLLOW && ...
    hypot(x - oldCorner(1), y - oldCorner(2)) > OLD_DMIN && ...
    hypot(x - oldCorner(1), y - oldCorner(2)) < OLD_DMAX;
back.use = ~first && useBehind;
back.dir = behind;
back.zone = BACK_ZONE;
uturn.on = uturnOn && state == S_FOLLOW;
uturn.from = uturnFrom;
uturn.dir = uturnDir;
old.use = oldUse;
old.path = oldPath;
old.len = oldLen;
old.tol = OLD_TOL;
posR = [x y Rm];
[found, dirW, uFound] = pickRun(occ, hit, cam.NB, psi, ref, ...
    back, uturn, posR, old);
if (isCircle || isBlob) && ~found, circCnt = circCnt + 1; else, circCnt = 0; end

switch state
    case S_TAKEOFF  % TAKEOFF: climb straight up over the start
        if all(tgt == 0), tgt = [x y]; startXY = [x y]; end
        hCmd = psi;
        crumb = [x y];
        pathXY(1,:) = [x y];
        nPath = 1;
        if z < HOVER_Z + 0.1
            state = S_FOLLOW;
        end
    case S_FOLLOW  % FOLLOW: fly to the point where the line leaves the ring, turn gradually to face it
        % nothing under us at the start: wrong spawn, ignore lines seen only in the ring
        if first && nCentre < START_MIN_N, found = false; end
        if found
            if lockOn && hypot(x - lockFrom(1), y - lockFrom(2)) < LOCK_D
                % committed turn: gradual corrections pass, sudden jumps do not
                if abs(wrapAngle(dirW - travel)) > LOCK_TOL, dirW = travel; end
            else
                lockOn = false;
                if ~first && abs(wrapAngle(dirW - travel)) > LOCK_TURN
                    lockOn = true;
                    nDir = 0;
                    lockFrom = [x y];
                    oldOn = true;
                    oldCorner = [x y];
                    oldPath = pathXY;
                    oldLen = nPath;   % remember the corner and the path flown on the leg we leave
                end
            end
            travel = dirW;
            dirHist = [dirW; dirHist(1:end-1)];
            nDir = min(nDir + 1, COAST_AVG);
            % Fast while the nose already points along the line, slow while it still has to turn
            align = max(0, cos(wrapAngle(dirW - psiNow)));
            if slowCnt > 0, align = 0; slowCnt = slowCnt - 1; end
            step = STEP + (STEP_MAX - STEP) * align*align;
            % Ring crossing as seen at image time: place the carrot from where the ring was seen
            xyImg = xyHist(LAG_N + 1, :);
            pCross = xyImg + Rm * [cos(dirW) sin(dirW)];
            dCross = hypot(x - pCross(1), y - pCross(2));
            if dCross > 0.05
                dirC = atan2(pCross(2) - y, pCross(1) - x);
            else
                dirC = dirW;
            end
            % Never swing the carrot more than 30 deg away from the measured crossing
            if abs(wrapAngle(dirC - dirW)) > 30*pi/180
                dirC = wrapAngle(dirW + max(-30*pi/180, min(30*pi/180, wrapAngle(dirC - dirW))));
            end
            tgt  = pointAlong(x, y, step, dirC);
            headingErr = wrapAngle(dirW - hCmd);
            if abs(headingErr) < YAW_SMALL
                yawGain = YAW_FINE;
            else
                yawGain = YAW_GAIN;
            end
            hCmd = wrapAngle(hCmd + max(-YAW_MAX, min(YAW_MAX, yawGain * headingErr)));
            first = false;
        elseif ~first                    % Nothing ahead: line ended (or a dropout), keep going straight
            state = S_COAST;
            coastFrom = [x y];
            coastN = max([nHist(end) nCentre]);
            slowCnt = SLOW_FRAMES;
            travel = coastHeading(travel, dirHist, nDir);
            tgt = pointAlong(x, y, COAST_STEP, travel);
        % No line yet at the start: after 1 s, creep toward where the app spawn bug puts the track start
        % (sim); on hardware SCENE_OFF is zero, so it searches around the takeoff point
        else
            [startCnt, srchOn, srchAng, tgt] = updateStartSearch(...
                startCnt, srchOn, srchAng, tgt, x, y, startXY, ...
                SCENE_OFF, SRCH_PITCH, SRCH_RMAX, SRCH_STEP, START_DELAY);
        end
    case S_COAST  % COAST: straight on along the last direction, looking for the landing circle
        if ~found && uFound && ~aheadRed && coastN > FAT_N && ...
                ~isCircle && ~isBlob && ...
                hypot(x - coastFrom(1), y - coastFrom(2)) > UTURN_D
            found = true;
            dirW = behind;
            uturnOn = true;
            uturnFrom = [x y];
            uturnDir = behind;
            crumb = [x y];
            pathXY(1,:) = [x y];
            nPath = 1;   % past a very sharp V tip: turn back, remember the incoming line
        end
        % Nothing ahead and a round red blob (or a blob after the gap) under us: the circle
        if ~found && (circCnt >= CIRC_FR ...
                || hypot(x - coastFrom(1), y - coastFrom(2)) > COAST_MAX)
            state = S_SETTLE;
            holdCnt = 0;
            tgt = [x y];
            centred = false;
            clipBias = [0 0];
        elseif found                                  % Line (or circle edge) ahead again
            state = S_FOLLOW;
            travel = dirW;
            nDir = 0;
            tgt = pointAlong(x, y, STEP, dirW);
        else
            tgt = pointAlong(x, y, COAST_STEP, travel);
        end
    case S_SETTLE  % SETTLE: centre over the circle (PI on the camera offset), then land
        % A line again and nothing round under us: not the circle (V tip), keep following
        if holdCnt < ABORT_FR && found && ~isCircle
            state = S_FOLLOW;
            travel = dirW;
            nDir = 0;
            tgt = pointAlong(x, y, STEP, dirW);
        else
            % The clip test runs only in SETTLE at hover height, where the centre disk covers the
            % whole circle; its bias is reused in LAND.
            if nCentre > LAND_MIN_N && still
                off0 = blobOffset(centre, psi, z, cam, CAM_FWD);
                [offC, clip] = clipCorrect(off0, centre, cam);
                if clip, clipBias = offC - off0; else, clipBias = [0 0]; end
                [tgt, landI, off] = landingCorrection(centre, psi, z, cam, CAM_FWD, x, y, landI, LAND_KI, clipBias);
                centred = hypot(off(1), off(2)) < SETTLE_TOL;
            end
            holdCnt = holdCnt + 1;
            if (holdCnt >= SETTLE_MIN && centred) || holdCnt >= SETTLE_MAX, state = S_LAND; end
        end
    case S_LAND  % LAND: lower slowly at a fixed point (soft landing)
        % clipBias (zero unless the circle was clipped in SETTLE) corrects the wall-cut centroid during descent.
        if nCentre > LAND_MIN_N && still && -z > LAND_MINH
            [tgt, landI, ~] = landingCorrection(centre, psi, z, cam, CAM_FWD, x, y, landI, LAND_KI, clipBias);
        end
        zCmd = min(zCmd + LAND_RATE, LAND_FLOOR);   % Gradual descent, ends just below floor level
    otherwise   % failsafe: an unexpected state value lands from the next frame
        state = S_LAND;
end

% localToNED rotates [x_b y_b] by the commanded heading, so pre-rotate the target back
traj = [ tgt(1)*cos(hCmd) + tgt(2)*sin(hCmd);
        -tgt(1)*sin(hCmd) + tgt(2)*cos(hCmd);
         zCmd;
         hCmd];
dbgDir = 99;
if found
    dbgDir = dirW;
end
nHist = [nHist(2:end) nCentre];
dbg = [state; hCmd; nCentre; dbgDir];
end

function [occ, hit, tot, hitAll, centre, metresPerPx] = scanImage(...
    mask, x, y, psi, z, cam, scene, OCC_FRAC)
% Scan image and compute ring occupancy, hit counts, central blob moments, and scale Kh.
% pixel -> metres at the current height
cosPsi = cos(psi);
sinPsi = sin(psi);
metresPerPx = cam.K * max(-z, 0.05) / (-cam.HOVER_Z);
sceneN = x + scene.OFF(1); sceneE = y + scene.OFF(2);                       % drone position in scene coordinates
% ---- one pass over the image: ring occupancy per angle bin + centre-disk centroid ----
hit = zeros(cam.NB,1); tot = zeros(cam.NB,1); hitAll = zeros(cam.NB,1);
sumF = 0; sumR = 0; sumFF = 0; sumRR = 0; sumFR = 0; nCentre = 0;
pN = 0; pE = 0;   % scene position of the current red pixel
minN = inf; maxN = -inf; minE = inf; maxE = -inf;
redM = false(120, 160);   % red pixels after the background-patch filter
for c = 1:160
    for r = 1:120
        f  = cam.RC - r;            % pixels forward of the drone
        rt = c - cam.CC;            % pixels right of the drone
        dist2 = f*f + rt*rt;
        red = mask(r,c);
        if red
            % x70: scene position of every red pixel (was computed only with USE_PATCH, so on hardware the
            % circle extent was 0, clipCorrect saw a "clipped" circle and sent the drone back toward the start)
            pN = sceneN + metresPerPx*(f*cosPsi - rt*sinPsi);
            pE = sceneE + metresPerPx*(f*sinPsi + rt*cosPsi);
            if scene.USE_PATCH                          % drop red that lies on the background patch (sim only)
                red = ~inPatch(pN, pE, scene.PATCH_E0, scene.PATCH_DE, scene.PATCH_N);
            end
        end
        redM(r,c) = red;
        if dist2 >= cam.R1*cam.R1 && dist2 <= cam.R2*cam.R2
            b = ringBin(f, rt, cam.NB);
            tot(b) = tot(b) + 1;
            if red, hitAll(b) = hitAll(b) + 1; end   % all red on the ring, connected or not
        elseif dist2 < cam.RIN*cam.RIN && red
            sumF = sumF + f; sumR = sumR + rt; nCentre = nCentre + 1;
            sumFF = sumFF + f*f; sumRR = sumRR + rt*rt; sumFR = sumFR + f*rt;
            if pN < minN, minN = pN; end
            if pN > maxN, maxN = pN; end
            if pE < minE, minE = pE; end
            if pE > maxE, maxE = pE; end
        end
    end
end
% ring hits only from red connected to the line under the drone (a neighbouring parallel line is ignored)
comp = connectedRed(redM, cam);
for c = 1:160
    for r = 1:120
        if comp(r,c)
            f = cam.RC - r; rt = c - cam.CC; dist2 = f*f + rt*rt;
            if dist2 >= cam.R1*cam.R1 && dist2 <= cam.R2*cam.R2
                b = ringBin(f, rt, cam.NB);
                hit(b) = hit(b) + 1;
            end
        end
    end
end
occ = hit > OCC_FRAC * tot;
centre.sumF = sumF;
centre.sumR = sumR;
centre.sumFF = sumFF;
centre.sumRR = sumRR;
centre.sumFR = sumFR;
centre.n = nCentre;
centre.minN = minN;
centre.maxN = maxN;
centre.minE = minE;
centre.maxE = maxE;
centre.sceneN = sceneN;
centre.sceneE = sceneE;
end

function [centreDir, minorStd, majorStd] = centreBlobShape(centre, psi)
% Compute blob centroid direction and narrowest/widest standard deviations.
sumF = centre.sumF;
sumR = centre.sumR;
sumFF = centre.sumFF;
sumRR = centre.sumRR;
sumFR = centre.sumFR;
nCentre = centre.n;
if nCentre > 0
    meanF = sumF/nCentre; meanR = sumR/nCentre;
    centreDir = wrapAngle(psi + atan2(meanR, meanF));
    % covariance of the red under the drone
    covFF = sumFF/nCentre - meanF*meanF; covRR = sumRR/nCentre - meanR*meanR; covFR = sumFR/nCentre - meanF*meanR;
    covMean = (covFF + covRR)/2; covHalfDiff = sqrt(max(0, ((covFF - covRR)/2)^2 + covFR^2));
    minorStd = sqrt(max(0, covMean - covHalfDiff));
    majorStd = sqrt(max(0, covMean + covHalfDiff));  % narrowest and widest spread, any direction
else
    centreDir = 0;
    minorStd = 0; majorStd = 0;
end
end

function [found, dirW, uFound] = pickRun(occ, hit, NB, psi, ref, ...
    back, uturn, posR, old)
% Each run of neighbouring occupied bins is one place where the line crosses the ring.
% Collect every crossing (wide runs give their two edges), then drop the one that points
% back at the breadcrumb (the line we came along) and pick the one closest to ref.
useBehind = back.use;
behind = back.dir;
backZone = back.zone;
avoidOn = uturn.on;
avoidA = uturn.from;
avoidDir = uturn.dir;
oldUse = old.use;
oldPath = old.path;
oldN = old.len;
oldTol = old.tol;
found = false;
dirW = 0;
uFound = false;   % a leg folding back beside the line we came along (very sharp V)
startBin = 0;
for b = 1:NB
    if ~occ(b), startBin = b; break; end
end
if startBin == 0, return; end            % whole ring red (too low / over a blob): no direction
candDir = zeros(2*NB, 1); nCand = 0;      % candidate world bearings
j = 1;
while j <= NB
    b = mod(startBin + j - 1, NB) + 1;
    if occ(b)
        runStart = j;
        while j <= NB && occ(mod(startBin + j - 1, NB) + 1), j = j + 1; end
        runEnd = j - 1;
        % one line is ~26 deg (5-6 bins) wide on the ring; a wider run holds several lines merged
        % (sharp corner or crossing), so judge every 5-bin window of it as its own candidate
        if runEnd - runStart + 1 <= 5, nWin = 1; else, nWin = runEnd - runStart - 3; end
        for k = 1:nWin
            if nWin == 1
                winStart = runStart;
                winEnd = runEnd;
            else
                winStart = runStart + k - 1;
                winEnd = winStart + 4;
            end
            sumCos = 0; sumSin = 0;
            for jj = winStart:winEnd
                bin = mod(startBin + jj - 1, NB) + 1;
                a  = -pi + (bin - 0.5) * 2*pi/NB;
                sumCos = sumCos + hit(bin)*cos(a); sumSin = sumSin + hit(bin)*sin(a);
            end
            nCand = nCand + 1;
            candDir(nCand) = wrapAngle(psi + atan2(sumSin, sumCos));
        end
    else
        j = j + 1;
    end
end
% the crossing closest to the breadcrumb is the line we came along; a sharp V's outgoing
% leg can also lie within backZone, so only that one (and anything very close) is dropped
backIdx = 0;
if useBehind
    bestBack = inf;
    for i = 1:nCand
        angOff = abs(wrapAngle(candDir(i) - behind));
        if angOff < bestBack, bestBack = angOff; backIdx = i; end
    end
    if bestBack >= backZone, backIdx = 0; end
end
% a candidate 6-25 deg off the breadcrumb (not the one on it) is a leg folding back beside ours
if useBehind && backIdx > 0
    foldOff = 6*pi/180;
    for i = 1:nCand
        angOff = abs(wrapAngle(candDir(i) - behind));
        if i ~= backIdx && angOff < backZone && angOff > foldOff
            foldOff = angOff; uFound = true;
        end
    end
end
% after a U-turn: of the crossings near the incoming line's direction, take the one whose ring
% point lies farthest from that line (the outgoing leg diverges, the incoming leg stays on it)
if avoidOn
    far = -1;
    for i = 1:nCand
        if abs(wrapAngle(candDir(i) - avoidDir)) < 45*pi/180   % 45 deg angle limit
            relX = posR(1) + posR(3)*cos(candDir(i)) - avoidA(1); relY = posR(2) + posR(3)*sin(candDir(i)) - avoidA(2);
            sideDist = abs(relX*sin(avoidDir) - relY*cos(avoidDir));
            if sideDist > far
                far = sideDist;
                dirW = candDir(i);
                found = true;
            end
        end
    end
    if found, return; end
end
best = inf;
for i = 1:nCand
    offBehind = abs(wrapAngle(candDir(i) - behind));
    if oldUse   % crossing on the path flown before the last sharp corner: that old leg, never take it
        ringX = posR(1) + posR(3)*cos(candDir(i)); ringY = posR(2) + posR(3)*sin(candDir(i)); onOld = false;
        for q = 1:oldN
            if hypot(ringX - oldPath(q,1), ringY - oldPath(q,2)) < oldTol, onOld = true; end
        end
        if onOld, continue; end
    end
    if i ~= backIdx && ~(useBehind && offBehind < backZone)
        cost = abs(wrapAngle(candDir(i) - ref));
        if cost < best, best = cost; dirW = candDir(i); found = true; end
    end
end
end

function p = pointAlong(x, y, dist, ang)
% Return 2D point at given distance and angle from (x, y).
p = [x + dist*cos(ang), y + dist*sin(ang)];
end

function b = ringBin(f, rt, NB)
% Compute ring angle bin index for given forward and right pixel offsets.
b = min(floor((atan2(rt, f) + pi) / (2*pi) * NB) + 1, NB);
end

function [tgt, landI, off] = landingCorrection(...
    centre, psi, z, cam, CAM_FWD, x, y, landI, LAND_KI, bias)
% Compute landing target position and update integral correction.
off = blobOffset(centre, psi, z, cam, CAM_FWD) + bias;
tgt = [x y] + off + landI;
landI = landI + LAND_KI * off;
end

function [off, clip] = clipCorrect(off, centre, cam)
% The cut side is the one the extent midpoint lies away from the centroid; the uncut edge plus the known radius gives the centre.
% shortcut: a circle clipped on both sides of one axis is not handled.
clip = false;
if centre.n > 0
    ext = centre.maxN - centre.minN;
    if ext < 2*cam.CIRC_R - cam.CLIP_TOL
        mid = (centre.minN + centre.maxN) / 2;
        cen = centre.sceneN + off(1);
        if cen > mid
            tru = centre.minN + cam.CIRC_R;
        else
            tru = centre.maxN - cam.CIRC_R;
        end
        off(1) = tru - centre.sceneN;
        clip = true;
    end
    ext = centre.maxE - centre.minE;
    if ext < 2*cam.CIRC_R - cam.CLIP_TOL
        mid = (centre.minE + centre.maxE) / 2;
        cen = centre.sceneE + off(2);
        if cen > mid
            tru = centre.minE + cam.CIRC_R;
        else
            tru = centre.maxE - cam.CIRC_R;
        end
        off(2) = tru - centre.sceneE;
        clip = true;
    end
end
end

function h = coastHeading(travel, dirHist, nDir)
% Coast along the mean of the recent line directions within 30 deg of the last one.
% One reading alone swings while the drone crosses the line (rand_s7_4: 0.22 rad off).
h = travel;
sx = 0; sy = 0; n = 0;
for i = 1:nDir
    if abs(wrapAngle(dirHist(i) - travel)) < 30*pi/180
        sx = sx + cos(dirHist(i));
        sy = sy + sin(dirHist(i));
        n = n + 1;
    end
end
if n >= 3, h = atan2(sy, sx); end
end

function [startCnt, srchOn, srchAng, tgt] = updateStartSearch(...
    startCnt, srchOn, srchAng, tgt, x, y, startXY, ...
    SCENE_OFF, SRCH_PITCH, SRCH_RMAX, SRCH_STEP, START_DELAY)
% Creep toward track start and execute spiral search if no line found.
startCnt = startCnt + 1;
if startCnt > START_DELAY
    srchCentre = startXY - SCENE_OFF;
    toCentre = srchCentre - [x y];
    distCentre = hypot(toCentre(1), toCentre(2));
    if distCentre <= SRCH_STEP, srchOn = true; end
    if srchOn
        srchR = SRCH_PITCH * srchAng / (2*pi);
        srchAng = srchAng + SRCH_STEP / max(srchR, SRCH_STEP);
        tgt = startXY + srchR * [cos(srchAng) sin(srchAng)];
        if srchR > SRCH_RMAX, tgt = [x y]; end
    else
        if distCentre > SRCH_STEP, tgt = [x y] + SRCH_STEP * toCentre / distCentre; end
    end
end
end

function [pathXY, nPath, crumb, behind, useBehind] = ...
    updateBreadcrumb(x, y, pathXY, nPath, NPATH, CRUMB, ...
    CRUMB_SEG, CRUMB_MIN_D)
% Update flown path points and compute breadcrumb position and incoming direction.
if nPath == 0 || hypot(x - pathXY(nPath,1), y - pathXY(nPath,2)) >= CRUMB_SEG
    if nPath == NPATH
        pathXY = [pathXY(2:end,:); [x y]];
    else
        nPath = nPath + 1;
        pathXY(nPath,:) = [x y];
    end
end
crumb = [x y];
acc = 0;
prevP = [x y];
for i = nPath:-1:1
    sg = hypot(prevP(1) - pathXY(i,1), prevP(2) - pathXY(i,2));
    if sg > 0 && acc + sg >= CRUMB
        crumb = prevP + (pathXY(i,:) - prevP) * ((CRUMB - acc) / sg);
        break;
    end
    acc = acc + sg;
    crumb = pathXY(i,:);
    prevP = pathXY(i,:);
end
crumbDist = hypot(x - crumb(1), y - crumb(2));
if crumbDist > CRUMB_MIN_D
    behind = atan2(crumb(2) - y, crumb(1) - x);
    useBehind = true;
else
    behind = 0;
    useBehind = false;
end
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end

function d = blobOffset(centre, psi, z, cam, CAM_FWD)
metresPerPx = cam.K * max(-z, 0.05) / (-cam.HOVER_Z);
fo = (centre.sumF / centre.n) * metresPerPx + CAM_FWD;
ri = (centre.sumR / centre.n) * metresPerPx;
d = [fo*cos(psi) - ri*sin(psi), fo*sin(psi) + ri*cos(psi)];
end

function tf = inPatch(north, east, E0, DE, PN)
% True if scene point (north, east) lies on the red background patch: North above the
% boundary PN, linearly interpolated between East samples E0, E0+DE, ...
k = (east - E0) / DE + 1;
k0 = min(max(floor(k), 1), numel(PN) - 1);
t  = min(max(k - k0, 0), 1);
tf = (k >= 0.5) && (k <= numel(PN) + 0.5) && (north > PN(k0) * (1 - t) + PN(k0 + 1) * t);
end

function comp = connectedRed(red, cam)
% Red pixels inside the look-ahead disk that connect to the red nearest the image centre.
% With no red near the centre (no line under the drone) every red pixel counts.
best = inf; seedR = 1; seedC = 1;
for c = 1:160
    for r = 1:120
        if red(r,c)
            f = cam.RC - r; rt = c - cam.CC; dist2 = f*f + rt*rt;
            if dist2 < best, best = dist2; seedR = r; seedC = c; end
        end
    end
end
if best > cam.RIN*cam.RIN
    comp = red;
    return;
end
comp = false(120, 160);
qr = zeros(19200, 1); qc = zeros(19200, 1);
head = 1; tail = 1; qr(1) = seedR; qc(1) = seedC; comp(seedR,seedC) = true;
while head <= tail
    r = qr(head); c = qc(head); head = head + 1;
    for k = 1:4
        if k == 1
            rr = r - 1; cc = c;
        elseif k == 2
            rr = r + 1; cc = c;
        elseif k == 3
            rr = r; cc = c - 1;
        else
            rr = r; cc = c + 1;
        end
        if rr >= 1 && rr <= 120 && cc >= 1 && cc <= 160
            if red(rr,cc) && ~comp(rr,cc)
                f = cam.RC - rr; rt = cc - cam.CC;
                if f*f + rt*rt <= cam.R2*cam.R2
                    tail = tail + 1; qr(tail) = rr; qc(tail) = cc; comp(rr,cc) = true;
                end
            end
        end
    end
end
end
