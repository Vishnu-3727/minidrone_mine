# Refine the new-leg direction in CORNER with the line fit (ring bins are 5 deg and noisy).
f = r'C:\Users\kkjjk\minidrone_mine\perception\trackGuidance.m'
s = open(f).read()
a = """            nextDir = cand(bi,1);
        end
        if ~isnan(nextDir), yC = yawToward(sp(4), nextDir, YAW_STEP); else, yC = sp(4); end   % smooth pivot"""
b = """            nextDir = cand(bi,1);
            % ring bins are 5 deg wide and the crossing is noisy: fit the new leg itself
            [ld, ~] = fitLine(pImg, nextDir, lineM, gN, gE);
            if ~isnan(ld) && abs(wrapA(ld - nextDir)) < 20*pi/180, nextDir = ld; end
        end
        if ~isnan(nextDir), yC = yawToward(sp(4), nextDir, YAW_STEP); else, yC = sp(4); end   % smooth pivot"""
assert s.count(a) == 1
s = s.replace(a, b)
open(f, 'w').write(s)
print('p1 applied')
