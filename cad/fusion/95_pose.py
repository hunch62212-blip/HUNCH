# 95_pose.py - 1) JOINT CHECK: drives the suspension and wheel joints made by 90_assembly_joints.py and compares every wheel centre
#    with the rocker-bogie kinematics (rocker about rb_P, bogie about rb_B, differential: left rocker = -right rocker, sim sign),
#    and spins wheels 90 deg (centre fixed, lateral axis).  Prints PASS / FAIL per test and "JOINT CHECK PASS|FAIL".
#    2) POSES: drives both arms and the ACM through the 5 poses (stow, pregrasp, pick, apex, place) and ends at stow.
# Poses are root-proxy transform overrides (lib apply_pose, which removes the joints: re-run 90 afterwards), joint angles come from
# the closed-form 2-link IK (lib arm_pose_q).  Change SEQ / RUN_POSES below, or use  `python3 tools/fusion_mcp.py pose <name>`
# for a single pose + `shot`.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

# Sim reference (src/sim/config.js ARM, arm.js fk), lengths / 8.16 (K): stow q = [2.8, -2.8, 1.2], wrist axis (161.56, 342.0) mm,
# psi 1.2 rad (hook seat (173.9, 373.7)); preGrasp seat (xBody - preFwd, TRUN_Y - preBelow) = (390, 70); grasp (420, 90);
# place q = [1.880, 2.113, 2.290] (q2 > 0 branch) at DECK.placeX -0.3474 m / 8.16 = -42.57 mm.
# CAD q differ slightly: the seat is hook_off ahead of the wrist.

SEQ = ['pick', 'pregrasp', 'apex', 'place', 'stow']
RUN_POSES = True            # False: joint check only (keeps the joints of 90)
TOL_MM = 0.05

SUSP = ('J_ROCKER_R', 'J_ROCKER_L', 'J_DIFF_R', 'J_DIFF_L', 'J_BOGIE_R', 'J_BOGIE_L')
WHEELS = ('FL', 'FR', 'ML', 'MR', 'RL', 'RR')
# (joints driven, in sim deg) -> expected (right rocker, right bogie, left bogie) in sim deg; left rocker = -right rocker
SUSP_TESTS = (
    ((('J_ROCKER_R', 10.0),), (10.0, 0.0, 0.0)),
    ((('J_DIFF_L', 8.0),), (-8.0, 0.0, 0.0)),
    ((('J_BOGIE_R', 15.0),), (0.0, 15.0, 0.0)),
    ((('J_BOGIE_L', -15.0),), (0.0, 0.0, -15.0)),
    ((('J_ROCKER_R', 10.0), ('J_BOGIE_R', 12.0)), (10.0, 12.0, 0.0)),
    ((('J_ROCKER_L', -11.0), ('J_BOGIE_L', 16.0)), (11.0, 0.0, 16.0)),
)
# (wheel joint, suspension joints set first): the wheel must turn 90 deg about the lateral axis with its centre fixed
SPIN_TESTS = (
    ('J_WHEEL_FR', ()),
    ('J_WHEEL_ML', ()),
    ('J_WHEEL_RR', (('J_ROCKER_R', 10.0), ('J_BOGIE_R', 12.0))),
    ('J_WHEEL_FL', (('J_ROCKER_R', 10.0),)),
)
RESET_ORDER = ('J_BOGIE_R', 'J_BOGIE_L', 'J_ROCKER_R', 'J_ROCKER_L', 'J_DIFF_R', 'J_DIFF_L') + tuple('J_WHEEL_' + k for k in WHEELS)


def rot(p, c, deg):
    """p rotated by deg about the sim z axis through c = (x, y); the z of p is kept."""
    a = math.radians(deg)
    dx, dy = p[0] - c[0], p[1] - c[1]
    return (c[0] + dx * math.cos(a) - dy * math.sin(a), c[1] + dx * math.sin(a) + dy * math.cos(a)) + tuple(p[2:])


def predict(p0, aR, bR, bL):
    """Wheel centres for right rocker aR (left rocker -aR) and bogies bR / bL (sim deg, about sim +z): F wheels ride on the rocker
    (about P), M / R wheels on the bogie: w' = rot(rot(w, B, b), P, a)."""
    Pp = (A.P('rb_P_x'), A.P('rb_P_y'))
    B = (A.P('rb_B_x'), A.P('rb_B_y'))
    out = {}
    for k, p in p0.items():
        a, b = (aR, bR) if k[1] == 'R' else (-aR, bL)
        out[k] = rot(p if k[0] == 'F' else rot(p, B, b), Pp, a)
    return out


def jt(name):
    try:
        return A.get_joint(name)
    except Exception:
        return None


def wheel_paths():
    wh = A.wheels()
    out = {}
    for o in A.find_prox('WHEEL_DRIVEN') + A.find_prox('WHEEL_IDLER'):
        p = A.occ_origin_sim(o)
        out[min(wh, key=lambda k: (wh[k]['x'] - p[0]) ** 2 + (wh[k]['y'] - p[1]) ** 2 + (wh[k]['z'] - p[2]) ** 2)] = o.fullPathName
    return out


def occs(paths):
    by = dict((o.fullPathName, o) for o in A.root().allOccurrences)
    return dict((k, by[fp]) for k, fp in paths.items())


def centres(paths):
    return dict((k, A.occ_origin_sim(o)) for k, o in occs(paths).items())


def rmat(o):
    m = o.transform2
    return [[m.getCell(r, c) for c in range(3)] for r in range(3)]


def check_joints():
    names = SUSP + tuple('J_WHEEL_' + k for k in WHEELS)
    missing = [n for n in names if jt(n) is None]
    if missing:
        print('JOINT CHECK SKIPPED: missing %s (run 90_assembly_joints.py first)' % ', '.join(missing))
        return None
    sg = dict((n, A.joint_sim_sign(jt(n))) for n in names)
    dirty = set()

    def setj(n, d):
        jt(n).jointMotion.rotationValue = math.radians(d) * sg[n]
        dirty.add(n)
        adsk.doEvents()

    def getj(n):
        return sg[n] * math.degrees(jt(n).jointMotion.rotationValue)

    def reset():
        for n in RESET_ORDER:
            if n in dirty:
                setj(n, 0.0)
        dirty.clear()

    print('suspension joints: limits (sim deg) and values')
    for n in SUSP:
        lm = jt(n).jointMotion.rotationLimits
        ls = '-'
        if lm.isMinimumValueEnabled and lm.isMaximumValueEnabled:
            ls = '%+.2f .. %+.2f' % tuple(sorted((sg[n] * math.degrees(lm.minimumValue), sg[n] * math.degrees(lm.maximumValue))))
        print('  %-10s limits %-16s value %+.3f' % (n, ls, getj(n)))
    paths = wheel_paths()
    if sorted(paths) != sorted(WHEELS):
        print('JOINT CHECK FAIL: wheels found %s' % sorted(paths))
        return False
    for n in names:
        if abs(getj(n)) > 1e-6:
            setj(n, 0.0)
    dirty.clear()
    p0 = centres(paths)
    tests = fails = 0

    for drive, (aR, bR, bL) in SUSP_TESTS:
        tests += 1
        label = ' + '.join('%s %+g' % d for d in drive)
        try:
            for n, d in drive:
                setj(n, d)
            p1 = centres(paths)
            pr = predict(p0, aR, bR, bL)
            err = max(math.dist(p1[k], pr[k]) for k in p1)
            moved = max(math.dist(p1[k], p0[k]) for k in p1)
            ok = err < TOL_MM
            print('%s %-34s expect rocker R %+g / L %+g, bogie R %+g / L %+g: max wheel error %.4f mm (wheels moved up to %.1f mm)'
                  % ('PASS' if ok else 'FAIL', label, aR, -aR, bR, bL, err, moved))
            print('     joints (sim deg): ' + '  '.join('%s %+.2f' % (n[2:], getj(n)) for n in SUSP))
            if not ok:
                fails += 1
                flip = predict(p0, -aR, -bR, -bL)
                if max(math.dist(p1[k], flip[k]) for k in p1) < TOL_MM:
                    print('     the wheels match the OPPOSITE angles: joint sign convention flipped')
                for k in sorted(p1):
                    print('     %s measured (%.2f, %.2f, %.2f) expected (%.2f, %.2f, %.2f)' % ((k,) + tuple(p1[k]) + tuple(pr[k])))
        except Exception as e:
            fails += 1
            print('FAIL %-34s error: %s' % (label, e))
        reset()
        back = centres(paths)
        res = max(math.dist(p0[k], back[k]) for k in p0)
        if res > TOL_MM:
            fails += 1
            print('FAIL reset after %s: wheels %.4f mm off the start' % (label, res))

    for wj, pre in SPIN_TESTS:
        tests += 1
        k = wj[-2:]
        label = wj + ((' after ' + ' + '.join('%s %+g' % d for d in pre)) if pre else '')
        try:
            for n, d in pre:
                setj(n, d)
            o = occs(paths)[k]
            c1, R1 = A.occ_origin_sim(o), rmat(o)
            setj(wj, 90.0)
            o = occs(paths)[k]
            c2, R2 = A.occ_origin_sim(o), rmat(o)
            M = [[sum(R2[r][i] * R1[c][i] for i in range(3)) for c in range(3)] for r in range(3)]   # R2 R1^T: the turn, world frame
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (M[0][0] + M[1][1] + M[2][2] - 1.0) / 2.0))))
            ax = (M[2][1] - M[1][2], M[0][2] - M[2][0], M[1][0] - M[0][1])
            lat = abs(ax[1]) / (math.sqrt(sum(v * v for v in ax)) or 1.0)                             # Fusion Y = sim lateral
            d = math.dist(c1, c2)
            ok = d < TOL_MM and abs(ang - 90.0) < 0.1 and lat > 0.999
            if not ok:
                fails += 1
            print('%s %-50s wheel turned %.2f deg, axis lateral %.5f, centre moved %.4f mm' % ('PASS' if ok else 'FAIL', label, ang, lat, d))
        except Exception as e:
            fails += 1
            print('FAIL %-50s error: %s' % (label, e))
        reset()
    back = centres(paths)
    res = max(math.dist(p0[k], back[k]) for k in p0)
    print('residual after reset: %.4f mm' % res)
    if res > TOL_MM:
        fails += 1
    print('JOINT CHECK %s (%d tests, %d failures)' % ('PASS' if fails == 0 else 'FAIL', tests, fails))
    return fails == 0


def run(_context: str):
    if not A.doc_ok():
        return
    check_joints()
    if not RUN_POSES:
        return
    for p in SEQ:
        q = A.apply_pose(p)
        print('pose %-9s q1,q2,q3 (deg) = %s   ACM base centre = %s' % (p, q, A.acm_pose_xy(p)))
