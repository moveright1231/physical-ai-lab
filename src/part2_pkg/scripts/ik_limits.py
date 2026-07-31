import math
from part2_pkg.ik_solve import ik

LIMITS = {
    'joint1': (-math.pi, math.pi),
    'joint2': (-1.5, 1.5),
    'joint3': (-1.5, 1.4),
    'joint4': (-1.7, 1.97),
}
NAMES = list(LIMITS)
TARGET = (0.15, 0.0, 0.15)

print(f'target {TARGET}')
print('pitch     q1        q2        q3        q4      verdict')
total = ok = 0
for deg in range(-90, 91, 10):
    sol = ik(*TARGET, math.radians(deg))
    if sol is None:
        print(f'{deg:>5}   (out of reach)')
        continue
    total += 1
    bad = [n for n, v in zip(NAMES, sol)
           if not (LIMITS[n][0] <= v <= LIMITS[n][1])]
    if bad:
        verdict = 'LIMIT ' + ','.join(bad)
    else:
        verdict = 'ok'
        ok += 1
    print(f'{deg:>5}  ' + ' '.join(f'{v:+.4f}' for v in sol) + f'   {verdict}')
print(f'\nmath solutions: {total}    actually executable: {ok}')
