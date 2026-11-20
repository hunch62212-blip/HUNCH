# 80_acm.py - the Arm-handled Cargo Module (ASM_ACM, root level so 95_pose can move it).  Local frame: origin = base centre on the
# ground/deck (x, z = 0), +y up, +x toward the rover front.  Built at the ground PICK pose (centre x = pick_x).
#   PRT_ACM_SHELL_A (lower, floor + walls + trunnion bosses + posts + pillars + keeper blocks) / _B (lid), split at
#   y = acm_trun_y + acm_split_dy by 4 tongue-and-groove keys + M3 bolts/inserts through the interior posts.
#   BUY_TRUNNION x2 (gold, d12), PRT_FOOT x4 (M3 into floor bosses), PRT_BUSH_ROUND / PRT_BUSH_SLOT (floor, x = +-70),
#   BUY_KEEPER (d6 rod behind the rear window), PRT_BALLAST_TRAY at acm_cg_y on 4 pillars, PRT_FIDUCIAL (AprilTag plate, rear face).
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    L, Wd, H, w = P('acm_L'), P('acm_W'), P('acm_H'), P('acm_wall')
    ft = P('acm_foot_h'); ty = P('acm_trun_y'); split_y = ty + P('acm_split_dy')
    hl, hw = L / 2, Wd / 2
    span = P('dk_pin_span')
    pick = A.mat(P('pick_x'), 0.0, 0.0)
    for o in list(A.root().occurrences):
        if o.component.name == 'ASM_ACM':
            o.deleteMe()
    top_occ = A.root().occurrences.addNewComponent(pick)
    top_occ.component.name = 'ASM_ACM'
    asm = top_occ.component

    # ---- shell -------------------------------------------------------------------------------------------
    sh = A.rbox_y(-hl, hl, ft, H, -hw, hw, 6.0)
    A.cut(sh, A.box(-hl + w, hl - w, ft + 3.0, H - w, -hw + w, hw - w))
    pd = P('acm_post_d')
    pcx, pcz = hl - w - pd / 2 - 0.5, hw - w - pd / 2 - 0.5
    posts = [(sx * pcx, sz * pcz) for sx in (-1, 1) for sz in (-1, 1)]
    for (x, z) in posts:
        A.union(sh, A.cyly(ft + 2.0, H - w + 0.5, x, z, pd / 2))
    for sz in (-1, 1):                                                   # trunnion bosses + press holes
        A.union(sh, A.cylz(sz * (hw - 13.0), sz * (hw - w + 0.5), 0, ty, 11.0))
        A.cut(sh, A.cylz(sz * (hw - 12.0), sz * (hw + 1.0), 0, ty, (P('trun_d') + 2 * P('clr_press')) / 2))
    pil = [(sx * 40.0, sz * 70.0) for sx in (-1, 1) for sz in (-1, 1)]
    ay = P('acm_cg_y')
    for (x, z) in pil:                                                   # ballast pillars
        A.union(sh, A.cyly(ft + 2.0, ay - 4.0, x, z, 5.0))
        A.cut(sh, A.insert_hole((x, ay - 4.0, z), (0, -1, 0), 'm3'))
    feet = [(sx * 55.0, sz * 90.0) for sx in (-1, 1) for sz in (-1, 1)]
    for (x, z) in feet:                                                  # foot screw bosses, insert from below
        A.union(sh, A.cyly(ft, ft + 14.0, x, z, 5.0))
        A.cut(sh, A.insert_hole((x, ft, z), (0, 1, 0), 'm3'))
    kx = -hl + 12.0; ky = 40.0; kd = P('keeper_d')
    for sz in (-1, 1):                                                   # keeper blocks
        A.union(sh, A.box(-hl + w - 0.5, kx + 8, ky - 9, ky + 9, sz * 22.0, sz * 36.0))
        A.cut(sh, A.cylz(sz * 20.0, sz * 36.5, kx, ky, kd / 2 + P('clr_press')))
    A.cut(sh, A.box(-hl - 1, -hl + w + 3, 49.0, 59.0, -22.0, 22.0))        # rear window for the deck-latch arm
    bd = P('dk_bush_d')
    for (x, bore_slot) in ((span / 2, False), (-span / 2, True)):         # floor holes for the bushings
        A.cut(sh, A.cyly(ft - 1, ft + 4, x, 0, 11.0))
    xs = (L - 20.0) / 2.0
    keys = [(x, z) for (x, z) in posts]
    lo, hi = A.split_keyed(sh, 'y', split_y, keys, key_w=10.0, key_h=6.0, key_depth=10.0, bolt_len=split_y - ft + 5.0)
    _, c = A.fresh_component(asm, 'PRT_ACM_SHELL_A')
    A.add_body(c, 'ACM_SHELL_A', lo, 'white', 'PETG 3 walls 20% infill + ballast; print upright')
    _, c = A.fresh_component(asm, 'PRT_ACM_SHELL_B')
    A.add_body(c, 'ACM_SHELL_B', hi, 'white', 'PETG 3 walls 20% infill; print flat')
    # ---- trunnions, feet, bushings, keeper, tray, fiducial -------------------------------------------------
    _, c = A.fresh_component(asm, 'BUY_TRUNNION')
    for sz, nm in ((1, 'TRUNNION_STBD'), (-1, 'TRUNNION_PORT')):
        A.add_body(c, nm, A.cylz(sz * (hw - 8.0), sz * (hw + P('trun_len')), 0, ty, P('trun_d') / 2), 'gold', 'steel rod d12 x 28 [VERIFY]')
    _, c = A.fresh_component(asm, 'PRT_FOOT')
    for i, (x, z) in enumerate(feet):
        f = A.cyly(0, ft, x, z, P('acm_foot') / 2)
        A.cut(f, A.hole((x, ft, z), (0, -1, 0), P('bolt_m3_clr'), ft))
        A.add_body(c, 'FOOT_%d' % (i + 1), f, 'tpu', 'TPU 95A')
    _, c = A.fresh_component(asm, 'PRT_BUSH_ROUND')
    b = A.cyly(ft, ft + 17.0, span / 2, 0, 11.0)
    A.union(b, A.cyly(ft + 3.0, ft + 5.0, span / 2, 0, 14.0))
    A.cut(b, A.cyly(ft - 1, ft + 18.0, span / 2, 0, bd / 2))
    A.cut(b, A.cone((span / 2, ft - 0.01, 0), 10.0, (span / 2, ft + 4.0, 0), bd / 2))
    A.add_body(c, 'BUSH_ROUND', b, 'red', 'PETG 100% infill; press-fit in floor')
    _, c = A.fresh_component(asm, 'PRT_BUSH_SLOT')
    sl = 6.0
    b = A.box(-span / 2 - 11 - sl / 2, -span / 2 + 11 + sl / 2, ft, ft + 17.0, -11, 11)
    A.cut(b, A.box(-span / 2 - bd / 2 - sl / 2, -span / 2 + bd / 2 + sl / 2, ft - 1, ft + 18.0, -bd / 2, bd / 2))
    A.add_body(c, 'BUSH_SLOT', b, 'red', 'PETG 100% infill; slot along the pin line (x)')
    _, c = A.fresh_component(asm, 'BUY_KEEPER')
    A.add_body(c, 'KEEPER_BAR', A.cylz(-36.0, 36.0, kx, ky, kd / 2), 'steel', 'steel rod d6 [VERIFY]')
    _, c = A.fresh_component(asm, 'PRT_BALLAST_TRAY')
    tr = A.box(-52, 52, ay - 4.0, ay, -88, 88)
    A.cut(tr, A.box(-46, 46, ay - 5.0, ay + 1.0, -82, 82)) if False else None
    for (x, z) in pil:
        A.cut(tr, A.hole((x, ay, z), (0, -1, 0), P('bolt_m3_clr'), 4.0))
    A.union(tr, A.box(-52, 52, ay, ay + 14.0, -88, -85))
    A.union(tr, A.box(-52, 52, ay, ay + 14.0, 85, 88))
    A.union(tr, A.box(-52, -49, ay, ay + 14.0, -88, 88))
    A.union(tr, A.box(49, 52, ay, ay + 14.0, -88, 88))
    A.add_body(c, 'BALLAST_TRAY', tr, 'petg', 'PETG; lead shot / steel plates to acm_mass')
    _, c = A.fresh_component(asm, 'PRT_FIDUCIAL')
    fz = P('fid_size') / 2 + 3
    A.add_body(c, 'FIDUCIAL_PLATE', A.box(-hl - 3.0, -hl, 138.0 - 2 * fz, 138.0, -fz, fz), 'black', 'PETG; AprilTag printed on paper, 60 mm')
    for o in asm.occurrences:
        A.report(o.component)
