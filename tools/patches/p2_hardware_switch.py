# HARDWARE switch: brick-patch filter off on a real floor (same as Vishnu's 956c7c3).
f = r'C:\Users\kkjjk\minidrone_mine\perception\trackGuidance.m'
s = open(f).read()
a = "PATCH_N    = [4.17 3.73 3.59 3.58 3.64 3.70 3.72 3.77 3.87 3.98 4.11 4.20 4.30];"
b = """HARDWARE   = false;        % <<< set true before deploying to the real drone (no brick patch on a real floor)
PATCH_N    = [4.17 3.73 3.59 3.58 3.64 3.70 3.72 3.77 3.87 3.98 4.11 4.20 4.30];
if HARDWARE, PATCH_N = 99 * ones(1, 13); end   % 99 m: outside any arena, so inPatch is never true"""
assert s.count(a) == 1
s = s.replace(a, b)
open(f, 'w').write(s)
print('p2 applied')
