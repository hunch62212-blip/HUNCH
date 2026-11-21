# 90_assembly_joints.py - as-built joints (articulation model), built at the arm STOW pose with the suspension flat.
#   revolute (21): J_WHEEL_FL/FR/ML/MR/RL/RR (wheel on its pod), J_SHOULDER_/J_ELBOW_/J_WRIST_ R/L (limits = sim ARM.limits as
#            deltas from stow), J_LATCH_R/L (gate on the hook), J_DECK_LATCH (vertical axis), J_ROCKER_R/L, J_DIFF_R/L, J_BOGIE_R/L
#   rigid:   servo/horn parts on their link (6 per arm); chassis-side parts on the grounded BUY_TUB (rocker pivot blocks + their 608s,
#            diff housing, arm towers, deck latch base); every ASM_DRIVE part at a wheel centre on its link (F pods -> rocker,
#            M / R pods -> bogie); rocker shaft + bogie pin -> rocker; bogie block, spacers, bogie 608s -> bogie
#   motion link ML_DIFFERENTIAL: the rockers counter-rotate relative to the body (J_DIFF_L = -J_DIFF_R in sim sign).
# Every revolute turns about its sketch-point normal (lib make_joint): lateral joints about sim z, the deck latch about sim y.
# Chained joints (bogie on rocker, wheel on pod, elbow / wrist / gate on the parent link) take their point from a 'J_PTS' sketch in
# the parent's own component (lib comp_joint_pt), so the axis travels with the parent.  Rocker / bogie limits = team-page GEO
# (public/team/js/rocker_bogie.js rhoMax 0.21 rad, betaMax 0.29 rad); src/sim has no rocker / bogie limit.
# Re-running deletes the motion links, joints and joint sketches first (lib clear_joints, clear_comp_joint_pts).  95_pose checks the
# suspension / wheel joints, then its poses (lib apply_pose) remove the joints, as does 99_verify: re-run 90 after them.
# The report lists any body-carrying part of ASM_SUSPENSION / ASM_DRIVE / ASM_WHEELS / ASM_ARM_* that no joint holds.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

SUSP_ROCKER_DEG = math.degrees(0.21)   # 12.03 deg, rocker vs body (GEO.rhoMax)
SUSP_BOGIE_DEG = math.degrees(0.29)    # 16.62 deg, bogie vs rocker (GEO.betaMax)

def first(name, pre=None):
    o = A.find_prox(name, pre)
    if not o:
        raise RuntimeError('90: component %s not found (run its build script first)' % name)
    return o[0]

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    r = A.root()
    A.apply_pose('stow')                       # the joints are defined at the stow pose; this also clears the old motion links + joints
    A.clear_joints()
    A.clear_comp_joint_pts()
    sk = A.joint_sketch()
    sky = A.joint_sketch_y()
    rev = []                                   # (joint, sim point its axis must pass through, vertical)
    cnt = {'rigid': 0}
    used = set()

    def uniq(name):
        n, i = name, 2
        while n in used:
            n = '%s_%d' % (name, i); i += 1
        used.add(n)
        return n

    def rj(name, mv, fx):
        A.make_joint(uniq(name), mv, fx); cnt['rigid'] += 1

    def rv(name, mv, fx, pt, xy, lo=None, hi=None, vertical=False):
        j = A.make_joint(uniq(name), mv, fx, pt, lo, hi, vertical)
        rev.append((j, (xy[0], xy[1]), vertical))
        return j

    # ---- ground: the tub is the body frame; chassis-side parts are tied to it ------------------------------------------------------
    tub = first('BUY_TUB')
    try:
        tub.isGrounded = True
    except Exception as e:
        print('WARNING BUY_TUB not grounded: %s' % e)
    for nm in ('PRT_JOINT_BLOCK_P_R', 'PRT_JOINT_BLOCK_P_L', 'BUY_608_P', 'BUY_DIFF_HOUSING', 'PRT_TOWER_R', 'PRT_TOWER_L', 'PRT_DECK_LATCH'):
        rj('RIGID_TUB_' + nm[4:], first(nm), tub)

    # ---- suspension: rocker on the tub-wall block, differential shaft in its housing, bogie on the rocker ----------------------------
    Pp = (P('rb_P_x'), P('rb_P_y')); Bp = (P('rb_B_x'), P('rb_B_y'))
    for tag in ('R', 'L'):
        rk, bg, shaft = first('BUY_ROCKER_' + tag), first('BUY_BOGIE_' + tag), first('BUY_ROCKER_SHAFT_' + tag)
        rv('J_ROCKER_' + tag, rk, first('PRT_JOINT_BLOCK_P_' + tag), A.joint_pt(sk, *Pp), Pp, -SUSP_ROCKER_DEG, SUSP_ROCKER_DEG)
        rj('RIGID_ROCKER_SHAFT_' + tag, shaft, rk)
        rv('J_DIFF_' + tag, shaft, first('BUY_DIFF_HOUSING'), A.joint_pt(sk, *Pp), Pp, -SUSP_ROCKER_DEG, SUSP_ROCKER_DEG)
        rj('RIGID_BOGIE_PIN_' + tag, first('BUY_BOGIE_PIN_' + tag), rk)
        rv('J_BOGIE_' + tag, bg, rk, A.comp_joint_pt(rk, *Bp), Bp, -SUSP_BOGIE_DEG, SUSP_BOGIE_DEG)
        rj('RIGID_BOGIE_BLOCK_' + tag, first('PRT_JOINT_BLOCK_B_' + tag), bg)
        rj('RIGID_BOGIE_SPACER_' + tag, first('PRT_BOGIE_SPACER_' + tag), bg)
        rj('RIGID_608_B_' + tag, first('BUY_608_B_' + tag), bg)
    n_ml = 0
    dR, dL = A.get_joint('J_DIFF_R'), A.get_joint('J_DIFF_L')
    try:
        mi = r.motionLinks.createInput(dR, dL)
        mi.valueOne = adsk.core.ValueInput.createByString('10 deg')
        mi.valueTwo = adsk.core.ValueInput.createByString('10 deg')
        mi.isReversed = (A.joint_sim_sign(dR) == A.joint_sim_sign(dL))      # sim sign: left = -right
        ml = r.motionLinks.add(mi)
        try:
            ml.name = 'ML_DIFFERENTIAL'
        except Exception:
            pass
        n_ml = 1
    except Exception as e:
        print('WARNING motion link ML_DIFFERENTIAL not created: %s' % e)

    # ---- pods ride on their link: every ASM_DRIVE part at a wheel centre -> rocker (F) or bogie (M, R) of that side -----------------
    wh = A.wheels()
    near = lambda p: min(wh, key=lambda n: (wh[n]['x'] - p[0]) ** 2 + (wh[n]['y'] - p[1]) ** 2 + (wh[n]['z'] - p[2]) ** 2)
    n_pod = dict((k, 0) for k in sorted(wh))
    for o in list(r.allOccurrences):
        ctx = o.assemblyContext
        if ctx is None or A.bn(ctx.component.name) != 'ASM_DRIVE':
            continue
        p = A.occ_origin_sim(o)
        k = near(p)
        if math.dist(p, (wh[k]['x'], wh[k]['y'], wh[k]['z'])) > 1.0:
            print('note: %s is not at a wheel centre, left untied' % o.fullPathName)
            continue
        link = ('BUY_ROCKER_' if wh[k]['link'] == 'rocker' else 'BUY_BOGIE_') + k[1]      # F pods on the rocker, M / R pods on the bogie
        rj('RIGID_POD_%s_%s' % (k, A.bn(o.component.name)[4:]), o, first(link))
        n_pod[k] += 1

    # ---- wheels: spin on their pod (motor mount / idler hub); the axis point sits at the pod origin = wheel centre -------------------
    pods = A.find_prox('PRT_MOTOR_MOUNT') + A.find_prox('PRT_IDLER_HUB')
    for o in A.find_prox('WHEEL_DRIVEN') + A.find_prox('WHEEL_IDLER'):
        k = near(A.occ_origin_sim(o))
        c = (wh[k]['x'], wh[k]['y'], wh[k]['z'])
        fx = min(pods, key=lambda b: math.dist(A.occ_origin_sim(b), c))
        rv('J_WHEEL_' + k, o, fx, A.comp_joint_pt(fx, *c), c)

    # ---- arms ----------------------------------------------------------------------------------------------------------------------
    qs = A.arm_pose_q('stow')
    mats, E, W = A.link_mats(qs)
    S = A.arm_geom()[0]
    g = A.hook_geom()
    a3 = qs[0] + qs[1] + qs[2]
    pv = (W[0] + g['pivot'][0] * math.cos(a3) - g['pivot'][1] * math.sin(a3), W[1] + g['pivot'][0] * math.sin(a3) + g['pivot'][1] * math.cos(a3))
    deg = math.degrees
    lim = [(deg(A.ARM_LIM[k][0]) - deg(qs[k]), deg(A.ARM_LIM[k][1]) - deg(qs[k])) for k in range(3)]
    for tag, pre in (('R', 'ASM_ARM_R'), ('L', 'ASM_ARM_L')):
        f = lambda n: first(n + '_' + tag, pre)
        for mv, fx in (('BUY_ELBOW_SERVO', 'PRT_UPPER_ARM'), ('BUY_SHOULDER_HORN', 'PRT_UPPER_ARM'), ('BUY_WRIST_SERVO', 'PRT_FORE_ARM'),
                       ('BUY_ELBOW_HORN', 'PRT_FORE_ARM'), ('BUY_LATCH_SERVO', 'PRT_HOOK'), ('BUY_WRIST_HORN', 'PRT_HOOK')):
            rj('RIGID_%s_%s' % (mv[4:], tag), f(mv), f(fx))
        ua, fa, hk, tw = f('PRT_UPPER_ARM'), f('PRT_FORE_ARM'), f('PRT_HOOK'), f('PRT_TOWER')
        rv('J_SHOULDER_' + tag, ua, tw, A.joint_pt(sk, S[0], S[1]), S, *lim[0])
        rv('J_ELBOW_' + tag, fa, ua, A.comp_joint_pt(ua, E[0], E[1]), E, *lim[1])
        rv('J_WRIST_' + tag, hk, fa, A.comp_joint_pt(fa, W[0], W[1]), W, *lim[2])
        rv('J_LATCH_' + tag, f(A.GATE), hk, A.comp_joint_pt(hk, pv[0], pv[1]), pv, -100, 5)
        for o in list(r.allOccurrences):       # any other arm part (apply_pose never moves it, e.g. a shoulder servo): tie it to the tower
            ctx = o.assemblyContext
            if ctx is None or ctx.assemblyContext is not None or A.bn(ctx.component.name) != pre:
                continue
            base = A.bn(o.component.name).rsplit('_', 1)[0]
            if base in A.LINK_OF or base in (A.GATE, 'PRT_TOWER') or o.component.bRepBodies.count == 0:
                continue
            rj('RIGID_TOWER_%s_%s' % (base[4:], tag), o, tw)
            print('note: %s tied to PRT_TOWER_%s' % (o.fullPathName, tag))

    # ---- deck latch (vertical axis) ------------------------------------------------------------------------------------------------
    rv('J_DECK_LATCH', first('PRT_DECK_LATCH_ARM'), first('PRT_DECK_LATCH'), A.joint_pt_y(sky, P('dk_latch_x'), 0.0),
       (P('dk_latch_x'), 0.0), -5, 100, vertical=True)

    # ---- report: axis direction, origin offset from the sim axis, limits in the sim sign ---------------------------------------------
    print('joints: %d revolute, %d rigid; total %d; motion links %d; BUY_TUB grounded %s; pod ties %s'
          % (len(rev), cnt['rigid'], len(A.all_joints()), n_ml, tub.isGrounded, n_pod))
    worst = 0.0
    for j, xy, vert in rev:
        v = j.jointMotion.rotationAxisVector
        s = 1 if (v.z if vert else -v.y) >= 0 else -1
        try:
            o = j.geometry.origin
            o = (o.x * 10.0, o.z * 10.0, -o.y * 10.0)
            e = math.hypot(o[0] - xy[0], (o[2] if vert else o[1]) - xy[1])
            os_ = '(%.2f, %.2f, %.2f)' % o
        except Exception:
            e, os_ = 0.0, 'n/a'
        worst = max(worst, e)
        lm = j.jointMotion.rotationLimits
        ls = '-'
        if lm.isMinimumValueEnabled and lm.isMaximumValueEnabled:
            a, b = sorted((s * deg(lm.minimumValue), s * deg(lm.maximumValue)))
            ls = '%+.2f..%+.2f' % (a, b)
        print('  %-13s axis(sim) (%+.3f, %+.3f, %+.3f)  origin(sim) %-26s off-axis %.3f mm  limits(sim deg) %s'
              % (j.name, v.x, v.z, -v.y, os_, e, ls))
    print('max joint-origin offset from its sim axis: %.3f mm' % worst)

    # ---- parts of the moving assemblies that no joint holds (they would not follow the motion) ----------------------------------------
    held = set([tub.fullPathName])
    for j in A.all_joints():
        for o in (j.occurrenceOne, j.occurrenceTwo):
            if o is not None:
                held.add(o.fullPathName)
    def is_held(fp):
        parts = fp.split('+')
        return any('+'.join(parts[:i]) in held for i in range(1, len(parts) + 1))
    free = [o.fullPathName for o in r.allOccurrences
            if A.bn(o.fullPathName.split(':')[0]) in ('ASM_SUSPENSION', 'ASM_DRIVE', 'ASM_WHEELS', 'ASM_ARM_R', 'ASM_ARM_L')
            and o.assemblyContext is not None and o.component.bRepBodies.count > 0 and not is_held(o.fullPathName)]
    print('parts in the moving assemblies held by no joint: %d %s' % (len(free), free[:12]))
