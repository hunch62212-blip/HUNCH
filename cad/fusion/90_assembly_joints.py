# 90_assembly_joints.py - as-built joints (articulation model), built at the arm STOW pose.
#   revolute: J_WHEEL_FL/FR/ML/MR/RL/RR (wheel on its pod), J_SHOULDER_/J_ELBOW_/J_WRIST_ R/L (limits as deltas from stow),
#   J_LATCH_R/L (gate on the hook), J_DECK_LATCH (vertical axis)   -> 15 revolute
#   rigid:    servo/horn parts fixed to their link (6 per arm).
# NOT jointed (granularity): rocker / bogie links live as bodies of one BUY_RB_LINK component - splitting it per link is P-next.
# Re-running deletes the joints and the J_POINTS sketches first.  95_pose / lib.apply_pose drive these joints.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def nearest(cands, p):
    return min(cands, key=lambda o: sum((a - b) ** 2 for a, b in zip(A.occ_origin_sim(o)[:2], p)))

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    r = A.root()
    for j in list(A.all_joints()):
        j.deleteMe()
    sk = A.joint_sketch()
    sky = A.joint_sketch_y()
    n_rev = n_rig = 0
    # ---- wheels ------------------------------------------------------------------------------------------
    wh = A.wheels()
    wocc = A.find_prox('WHEEL_DRIVEN') + A.find_prox('WHEEL_IDLER')
    for o in wocc:
        p = A.occ_origin_sim(o)
        key = min(wh, key=lambda k: (wh[k]['x'] - p[0]) ** 2 + (wh[k]['y'] - p[1]) ** 2 + (wh[k]['z'] - p[2]) ** 2)
        w = wh[key]
        base = A.find_prox('PRT_MOTOR_MOUNT') if w['kind'] == 'drive' else A.find_prox('PRT_IDLER_HUB')
        fx = min(base, key=lambda b: sum((a - c) ** 2 for a, c in zip(A.occ_origin_sim(b), (w['x'], w['y'], w['z']))))
        A.make_joint('J_WHEEL_' + key, o, fx, A.joint_pt(sk, w['x'], w['y'])); n_rev += 1
    # ---- arms ----------------------------------------------------------------------------------------------
    qs = A.arm_pose_q('stow')
    mats, E, W = A.link_mats(qs)
    S = A.arm_geom()[0]
    g = A.hook_geom()
    a3 = qs[0] + qs[1] + qs[2]
    pv = (W[0] + g['pivot'][0] * math.cos(a3) - g['pivot'][1] * math.sin(a3), W[1] + g['pivot'][0] * math.sin(a3) + g['pivot'][1] * math.cos(a3))
    deg = math.degrees
    for tag, pre in (('R', 'ASM_ARM_R'), ('L', 'ASM_ARM_L')):
        f = lambda n: A.find_prox(n + '_' + tag, pre)[0]
        for mv, fx in (('BUY_ELBOW_SERVO', 'PRT_UPPER_ARM'), ('BUY_SHOULDER_HORN', 'PRT_UPPER_ARM'), ('BUY_WRIST_SERVO', 'PRT_FORE_ARM'),
                       ('BUY_ELBOW_HORN', 'PRT_FORE_ARM'), ('BUY_LATCH_SERVO', 'PRT_HOOK'), ('BUY_WRIST_HORN', 'PRT_HOOK')):
            A.make_joint('RIGID_%s_%s' % (mv[4:], tag), f(mv), f(fx)); n_rig += 1
        A.make_joint('J_SHOULDER_' + tag, f('PRT_UPPER_ARM'), f('PRT_TOWER'), A.joint_pt(sk, S[0], S[1]), -30 - deg(qs[0]), 170 - deg(qs[0])); n_rev += 1
        A.make_joint('J_ELBOW_' + tag, f('PRT_FORE_ARM'), f('PRT_UPPER_ARM'), A.joint_pt(sk, E[0], E[1]), -90 - deg(qs[1]), 145 - deg(qs[1])); n_rev += 1
        A.make_joint('J_WRIST_' + tag, f('PRT_HOOK'), f('PRT_FORE_ARM'), A.joint_pt(sk, W[0], W[1]), 70 - deg(qs[2]), 215 - deg(qs[2])); n_rev += 1
        A.make_joint('J_LATCH_' + tag, f('PRT_LATCH_GATE'), f('PRT_HOOK'), A.joint_pt(sk, pv[0], pv[1]), -100, 5); n_rev += 1
    arm_o = A.find_prox('PRT_DECK_LATCH_ARM')[0]; base_o = A.find_prox('PRT_DECK_LATCH')[0]
    A.make_joint('J_DECK_LATCH', arm_o, base_o, A.joint_pt_y(sky, P('dk_latch_x'), 0.0), -5, 100, vertical=True); n_rev += 1
    print('joints: %d revolute, %d rigid; total %d' % (n_rev, n_rig, len(A.all_joints())))
