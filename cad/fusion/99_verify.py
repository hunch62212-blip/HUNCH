# 99_verify.py - full verification (prints PASS/FAIL lines; run after any rebuild).  Ends with the arms at STOW and the as-built joints deleted
# (re-run 90_assembly_joints.py afterwards if the joint model is wanted).
#   1 naming            2 bed fit + mass + inserts per PRT_ body      3 keep-out intrusion by non-arm bodies (REF_SWEEP)
#   4 arm extent vs REF_SWEEP at the 5 poses                          5 interference arm vs rest at the 5 poses (+ ACM vs arm)
#   6 CG vs plan 3 (Option R): curb x = +9.7, loaded x ~ -4 (+-10 mm)  7 totals
import math, time
import adsk.core, adsk.fusion
import aegis_cad as A

POSES = ['stow', 'pregrasp', 'pick', 'apex', 'place']
ARM_PREFIX = ('ASM_ARM_R:', 'ASM_ARM_L:')

ARM_NAMES = ('PRT_TOWER', 'BUY_SHOULDER', 'PRT_UPPER', 'BUY_ELBOW', 'PRT_FORE', 'BUY_WRIST', 'PRT_HOOK', 'PRT_LATCH_GATE', 'BUY_LATCH_SERVO')

def is_arm(ent):
    n = A.bn(ent.parentComponent.name)
    return n.startswith(ARM_NAMES) and n[-2:] in ('_R', '_L')

def bb_sim(b):
    lo, hi = b.boundingBox.minPoint, b.boundingBox.maxPoint
    zs = sorted((-hi.y * 10, -lo.y * 10))
    return (lo.x * 10, hi.x * 10, zs[0], zs[1], lo.z * 10, hi.z * 10)     # x0,x1, z0,z1 (sim), y0,y1 (sim)

def arm_bodies():
    out = []
    for o in A.root().allOccurrences:
        if o.fullPathName.startswith(ARM_PREFIX) and o.assemblyContext is not None:
            out += [(o, b) for b in o.bRepBodies]
    return out

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    fails = 0
    A.clear_joints()            # motion links + as-built joints (the poses below re-place the links): re-run 90 afterwards
    # ---------------------------------------------------------------- 1,2 naming / bed / mass / inserts
    seen = {}
    for o in A.root().allOccurrences:
        seen.setdefault(A.bn(o.component.name), o.component)
    tot_mass = 0.0; tot_ins = 0; n_prt = 0
    for name, comp in sorted(seen.items()):
        if not name.startswith(('PRT_', 'BUY_', 'REF_', 'ASM_', 'WHEEL_')):
            print('FAIL naming:', name); fails += 1
        if not name.startswith('PRT_'):
            continue
        for b in comp.bRepBodies:
            n_prt += 1
            d = A.dims_sorted(b)
            fit = A.bed_fits(d)
            mass = b.volume * P('rho_petg') * P('infill_eff')
            ins = sum(int(b.attributes.itemByName('aegis', 'n_' + k).value) for k in ('m3', 'm25', 'm4') if b.attributes.itemByName('aegis', 'n_' + k))
            tot_mass += mass; tot_ins += ins
            if not fit:
                fails += 1
                print('FAIL bed   %-34s dims %s' % (name + '/' + b.name, ['%.0f' % x for x in d]))
    print('%s naming/bed: %d PRT_ bodies, mass of one of each %.0f g, heat-set inserts %d' % ('PASS' if not fails else 'FAIL', n_prt, tot_mass, tot_ins))
    # ---------------------------------------------------------------- 3 keep-out intrusion by non-arm bodies
    sx0, sx1, hw, sy1 = P('sweep_x0'), P('sweep_x1'), P('sweep_halfW'), P('sweep_top')
    sy0 = P('ch_deckY') + 10.0
    bad = []
    for o in A.root().allOccurrences:
        top = o.fullPathName.split(':')[0]
        if top.startswith(('REF_', 'ASM_ARM', 'ASM_DECK', 'ASM_ACM')) or o.assemblyContext is None and False:
            continue
        for b in o.bRepBodies:
            x0, x1, z0, z1, y0, y1 = bb_sim(b)
            if x0 < sx1 and x1 > sx0 and z0 < hw and z1 > -hw and y0 < sy1 and y1 > sy0:
                bad.append(A.bn(o.component.name) + '/' + b.name)
    print('%s keep-out (REF_SWEEP) intrusions by non-arm bodies: %s' % ('PASS' if not bad else 'FAIL', bad if bad else 'none'))
    fails += bool(bad)
    # ---------------------------------------------------------------- 4,5 poses: extent and interference
    t_all = time.time()
    for pose in POSES:
        A.apply_pose(pose)
        ab = arm_bodies()
        ext = [1e9, -1e9, 1e9, -1e9, 1e9, -1e9]
        for o, b in ab:
            bb = bb_sim(b)
            for k in (0, 2, 4):
                ext[k] = min(ext[k], bb[k]); ext[k + 1] = max(ext[k + 1], bb[k + 1])
        carry = pose in ('stow', 'apex', 'place')
        inside = ext[0] >= sx0 - 1 and ext[1] <= sx1 + 100 and ext[5] <= sy1 + 1
        print('%s extent %-8s arm x %.0f..%.0f  |z| %.0f..%.0f  y %.0f..%.0f  (sweep x %.0f..%.0f top %.0f%s)' % (
            'PASS' if (inside or not carry) else 'FAIL', pose, ext[0], ext[1], min(abs(ext[2]), abs(ext[3])), max(abs(ext[2]), abs(ext[3])), ext[4], ext[5], sx0, sx1, sy1,
            '' if carry else '; ground-reach pose, info only'))
        if carry and not inside:
            fails += 1
        # interference: arm bodies vs all other bodies whose bbox overlaps the arm bbox (+ACM), pairwise
        coll = adsk.core.ObjectCollection.create()
        armset = set(); n = 0
        cand = []
        for o in A.root().allOccurrences:
            if o.assemblyContext is None:
                continue
            nm = o.fullPathName
            if nm.startswith(('REF_', 'PRT_FIT_COUPON')):
                continue
            for b in o.bRepBodies:
                if not b.isVisible and False:
                    continue
                isarm = nm.startswith(ARM_PREFIX)
                bb = bb_sim(b)
                if not isarm:
                    near = False
                    for oo, ba in ab:
                        a = bb_sim(ba)
                        if bb[0] < a[1] and bb[1] > a[0] and bb[2] < a[3] and bb[3] > a[2] and bb[4] < a[5] and bb[5] > a[4]:
                            near = True; break
                    if not near:
                        continue
                cand.append((isarm, o, b))
        for isarm, o, b in cand:
            coll.add(b)
            if isarm:
                armset.add(b.entityToken)
        hits = []
        if coll.count > 1:
            inp = A.design().createInterferenceInput(coll)
            inp.areCoincidentFacesInterference = False
            res = A.design().analyzeInterference(inp)
            for i in range(res.count):
                r = res.item(i)
                e1, e2 = r.entityOne, r.entityTwo
                t1 = is_arm(e1); t2 = is_arm(e2)
                if not (t1 or t2):
                    continue
                vol = 0.0
                try:
                    vol = r.interferenceBody.volume
                except Exception:
                    pass
                n1 = A.bn(e1.parentComponent.name); n2 = A.bn(e2.parentComponent.name)
                hits.append((vol, '%s/%s x %s/%s' % (n1, e1.name, n2, e2.name)))
        # classify: arm-arm self hits (parts of one link overlapping by design are listed as INFO), arm vs others = FAIL unless trunnion/hook contact
        bad = [h for h in hits if h[0] > 0.02]
        self_hits = [h for h in bad if all(x.split('/')[0].startswith(ARM_NAMES) and x.split('/')[0][-2:] in ('_R', '_L') for x in h[1].split(' x '))]
        env_hits = [h for h in bad if h not in self_hits]
        print('%s interference %-8s bodies %d, arm-vs-rest overlaps (>0.02 cm3): %d, arm self/joint overlaps: %d, %.0f s' % (
            'PASS' if not env_hits else 'FAIL', pose, coll.count, len(env_hits), len(self_hits), time.time() - t_all))
        for v, s in sorted(env_hits, reverse=True)[:12]:
            print('      %.2f cm3  %s' % (v, s))
        for v, s in sorted(self_hits, reverse=True)[:4]:
            print('      (self) %.2f cm3  %s' % (v, s))
        fails += bool(env_hits)
    # ---------------------------------------------------------------- 6 CG vs plan 3
    A.apply_pose('stow')
    groups = {'ASM_CHASSIS': 2.00, 'ASM_SUSPENSION': 1.50 * 0.35, 'ASM_WHEELS': 1.50 * 0.65, 'ASM_DRIVE': 0.84 + 0.12, 'ASM_DECK_ACM': 0.10}
    bat, ele = 1.65, 0.45
    arm_fixed, arm_mov = 0.44, 0.80
    pts = []                                    # (mass, x, y)

    def com_of(occs):
        v = 0.0; cx = 0.0; cy = 0.0; cz = 0.0
        for o in occs:
            for b in o.bRepBodies:
                vol = b.volume
                p = b.physicalProperties.centerOfMass
                v += vol; cx += vol * p.x * 10; cy += vol * p.z * 10; cz += vol * (-p.y) * 10
        return (cx / v, cy / v, cz / v) if v else (0, 0, 0)

    tops = {}
    for o in A.root().allOccurrences:
        if o.assemblyContext is None:
            continue
        tops.setdefault(o.fullPathName.split(':')[0], []).append(o)
    for g, m in groups.items():
        if g in tops:
            c = com_of(tops[g]); pts.append((m, c[0], c[1], c[2]))
    sn = tops.get('ASM_SENSORS', [])          # plan: nose LiDAR + cams (front) 0.30 kg, rear mast + LiDAR 0.10 kg
    rear = [o for o in sn if 'REAR' in A.bn(o.component.name)]
    front = [o for o in sn if o not in rear]
    if front: c = com_of(front); pts.append((0.30, c[0], c[1], c[2]))
    if rear: c = com_of(rear); pts.append((0.10, c[0], c[1], c[2]))
    pw = tops.get('ASM_POWER', [])
    bo = [o for o in pw if A.bn(o.component.name) in ('BUY_BATTERY', 'PRT_BATTERY_TRAY', 'PRT_BATTERY_LID')]
    eo = [o for o in pw if o not in bo]
    if bo: c = com_of(bo); pts.append((bat, c[0], c[1], c[2]))
    if eo: c = com_of(eo); pts.append((ele, c[0], c[1], c[2]))
    arm = tops.get('ASM_ARM_R', []) + tops.get('ASM_ARM_L', [])
    fixed = [o for o in arm if A.bn(o.component.name).startswith(('PRT_TOWER', 'BUY_SHOULDER_SERVO'))]
    moving = [o for o in arm if o not in fixed]
    if fixed: c = com_of(fixed); pts.append((arm_fixed, c[0], c[1], c[2]))
    if moving: c = com_of(moving); pts.append((arm_mov, c[0], c[1], c[2]))
    M = sum(p[0] for p in pts); xc = sum(p[0] * p[1] for p in pts) / M
    yc = sum(p[0] * p[2] for p in pts) / M
    for p, lab in zip(pts, [None] * len(pts)):
        print('      CG item m=%.2f kg x=%+.0f y=%.0f' % (p[0], p[1], p[2]))
    target = 9.7
    ok = abs(xc - target) <= 10.0
    print('%s CG curb (stowed): x = %+.1f mm  y = %.0f mm  mass %.2f kg  (plan Option R: x = +9.7 +-10; plan curb 8.30 kg)' % ('PASS' if ok else 'FAIL', xc, yc, M))
    fails += not ok
    ax, ay = A.acm_pose_xy('place')
    ml = M + 1.0
    xl = (xc * M + 1.0 * ax) / ml
    ok = abs(xl) <= 0.02 * 360 + 10.0
    print('%s CG loaded (ACM on deck at x = %.0f): x = %+.1f mm  (plan: -4 mm; rule 0 +-%.0f)' % ('PASS' if ok else 'FAIL', ax, xl, 0.02 * 360 + 10))
    fails += not ok
    front = (xl + 120.0) / 360.0
    print('INFO  flat-ground front-wheel share (plan 3.1: f = (x+120)/360) loaded: %.1f %%' % (front * 100))
    print('SUMMARY fails=%d  poses reset to stow; joints removed (re-run 90_assembly_joints.py)' % fails)
