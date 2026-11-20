# 30_rocker_bogie.py - ASM_SUSPENSION: rocker-bogie from the GEO params (rb_P = rocker pivot, rb_B = bogie pivot).
#   BUY_RB_LINK        12 x 18 aluminium bars: rocker P->F and P->B (z 169..181), bogie B->M and B->R (z 153..165),
#                      drilled: pivot holes, M4 pod-ear holes at every wheel end, M3 cross-bolt holes in the bogie ends
#   PRT_JOINT_BLOCK    2 x rocker pivot block (on the tub wall, 4 x M3 inserts) + 2 x bogie block (bearing seats both faces,
#                      two bar tunnels with clr_slip fit and M3 cross bolts into heat-set inserts)
#   PRT_BOGIE_SPACER   4 x 16 mm riser between bogie bar and pod ear (so every pod sits on the z = rb_face_z plane)
#   BUY_608            8 x 608 (2 per pivot), BUY_ROCKER_SHAFT x2 + BUY_DIFF_HOUSING (differential placeholder), BUY_BOGIE_PIN x2
# Bogie plane centre rb_bogie_z = 159 (sim 157 +2 mm so the joint block keeps >= 1.8 mm walls) - noted in params.csv.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_SUSPENSION')
    W = A.wheels()
    Px, Py = P('rb_P_x'), P('rb_P_y'); Bx, By = P('rb_B_x'), P('rb_B_y')
    lt, lh = P('rb_link_t'), P('rb_link_h')
    zr, zb, zf = P('rb_rocker_z'), P('rb_bogie_z'), P('rb_face_z')
    off, pitch = P('mot_ear_off'), P('mot_ear_pitch')
    slip = P('clr_slip'); r = P('wh_r')
    F = (P('wb_x'), r); M = (0.0, r); R = (-P('wb_x'), r)
    jb_t, jb_w = P('jb_t'), P('jb_w')
    stub = P('jb_stub')
    ang = lambda a, b: math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    uM = ang((Bx, By), M); uR = ang((Bx, By), R)          # directions B -> M, B -> R
    tz = P('ch_tubZ') + P('ch_tub_t')                     # tub wall outer face (150)
    zbl0, zbl1 = zb - lt / 2, zb + lt / 2                 # bogie bar z-range (153..165)

    def ear_holes(body, wheel_xy, to_pivot_ang, z0, z1, s):
        d = A.dir_xy(to_pivot_ang)
        for k in range(2):
            q = (wheel_xy[0] + d[0] * (off + k * pitch), wheel_xy[1] + d[1] * (off + k * pitch))
            A.cut(body, A.hole((q[0], q[1], s * z0), (0, 0, s), P('bolt_m4_clr'), z1 - z0 + 1, over=1.0))

    # ---------------------------------------------------------------- bars
    _, c = A.fresh_component(asm, 'BUY_RB_LINK')
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        a0, a1 = zr - lt / 2, zr + lt / 2
        rk = A.bar_xy(P_xy := (Px, Py), F, lh, s * a0, s * a1)
        A.union(rk, A.bar_xy((Px, Py), (Bx, By), lh, s * a0, s * a1))
        A.cut(rk, A.hole((Px, Py, s * a0), (0, 0, s), P('rb_pin_d'), lt, over=1))
        A.cut(rk, A.hole((Bx, By, s * a0), (0, 0, s), P('rb_pin_d'), lt, over=1))
        ear_holes(rk, F, ang(F, (Px, Py)), a0, a1, s)
        A.add_body(c, 'ROCKER_' + sn, rk, 'alu', '6061 flat bar 12x18, bent-link stand-in')
        # bogie: bars end inside the block tunnels, 14 mm from B
        bg = None
        for (W_, uu) in ((M, uM), (R, uR)):
            d = A.dir_xy(uu)
            e = (Bx + d[0] * 14.0, By + d[1] * 14.0)
            bar = A.bar_xy(e, W_, lh, s * zbl0, s * zbl1)
            ear_holes(bar, W_, uu + 180.0, zbl0, zbl1, s)
            n = (-d[1], d[0])
            for sc in (26.0, 40.0):
                q = (Bx + d[0] * sc, By + d[1] * sc, s * zb)
                A.cut(bar, A.cyl((q[0] - n[0] * 12, q[1] - n[1] * 12, q[2]), (q[0] + n[0] * 12, q[1] + n[1] * 12, q[2]), P('bolt_m3_clr') / 2))
            bg = bar if bg is None else A.union(bg, bar)
        A.add_body(c, 'BOGIE_' + sn, bg, 'alu', '6061 flat bar 12x18 (two arms)')

    # ---------------------------------------------------------------- joint blocks
    _, c = A.fresh_component(asm, 'PRT_JOINT_BLOCK')
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        # rocker pivot block on the tub wall: z tz .. tz + jb_t
        z0, z1 = tz, tz + jb_t
        b = A.rbox_z(Px - 20, Px + 20, Py - 20, Py + 20, s * z0, s * z1, 6.0)
        A.bearing_seat_cut(b, (Px, Py, s * z0), (0, 0, s))
        A.bearing_seat_cut(b, (Px, Py, s * z1), (0, 0, -s))
        for dx in (-14, 14):
            for dy in (-14, 14):
                A.cut(b, A.insert_hole((Px + dx, Py + dy, s * z0), (0, 0, s), 'm3'))
        A.add_body(c, 'JOINT_BLOCK_P_' + sn, b, 'petg', 'PETG 6 walls')
        # bogie pivot block: z 151 .. 169
        z0 = zb - lt / 2 - 3.0 + 0.0 - 0.0 + 0.0
        z0, z1 = zbl0 - 2.0, zbl0 - 2.0 + jb_t
        b = A.cylz(s * z0, s * z1, Bx, By, jb_w / 2)
        for (uu) in (uM, uR):
            d = A.dir_xy(uu)
            A.union(b, A.bar_xy((Bx, By), (Bx + d[0] * stub, By + d[1] * stub), lh + 2 * slip + 2 * P('wall_min'), s * z0, s * z1))
        for uu in (uM, uR):
            d = A.dir_xy(uu); n = (-d[1], d[0])
            for sc in (26.0, 40.0):
                q = (Bx + d[0] * sc, By + d[1] * sc)
                # insert boss on the +n side of the tunnel
                A.union(b, A.cyl((q[0] + n[0] * 8, q[1] + n[1] * 8, s * zb), (q[0] + n[0] * 18, q[1] + n[1] * 18, s * zb), P('ins_m3_boss') / 2 + 1))
        for uu in (uM, uR):
            d = A.dir_xy(uu); n = (-d[1], d[0])
            A.cut(b, A.rect_xy((Bx, By), uu, 14.0, stub + 1, lh + 2 * slip, s * (zb - lt / 2 - slip), s * (zb + lt / 2 + slip)))
            for sc in (26.0, 40.0):
                q = (Bx + d[0] * sc, By + d[1] * sc, s * zb)
                A.cut(b, A.cyl((q[0] - n[0] * 14, q[1] - n[1] * 14, q[2]), (q[0] - n[0] * 8, q[1] - n[1] * 8, q[2]), P('bolt_m3_clr') / 2))
                ins = A.insert_hole((q[0] + n[0] * (lh / 2 + slip), q[1] + n[1] * (lh / 2 + slip), q[2]), (n[0], n[1], 0), 'm3')
                A.cut(b, ins)
        A.bearing_seat_cut(b, (Bx, By, s * z0), (0, 0, s))
        A.bearing_seat_cut(b, (Bx, By, s * z1), (0, 0, -s))
        A.add_body(c, 'JOINT_BLOCK_B_' + sn, b, 'petg', 'PETG 6 walls')

    # ---------------------------------------------------------------- spacers
    _, c = A.fresh_component(asm, 'PRT_BOGIE_SPACER')
    for k in ('ML', 'MR', 'RL', 'RR'):
        w = W[k]; s = w['side']
        d = A.dir_xy(w['arm_ang'])
        end = (w['x'] + d[0] * (off + pitch + 9), w['y'] + d[1] * (off + pitch + 9))
        b = A.bar_xy((w['x'], w['y']), end, lh, s * zbl1, s * zf)
        for q_ in range(2):
            q = (w['x'] + d[0] * (off + q_ * pitch), w['y'] + d[1] * (off + q_ * pitch))
            A.cut(b, A.hole((q[0], q[1], s * zbl1), (0, 0, s), P('bolt_m4_clr'), zf - zbl1 + 1, over=1.0))
        A.add_body(c, 'BOGIE_SPACER_' + k, b, 'petg', 'PETG 5 walls')

    # ---------------------------------------------------------------- bought pivots
    _, c = A.fresh_component(asm, 'BUY_608')
    bw = P('brg_w')
    def brg(s, x, y, zc):
        return A.tube_z(s * zc, s * (zc + bw), x, y, P('brg_od') / 2, P('brg_id') / 2)
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        A.add_body(c, 'BRG_P_OUT_' + sn, brg(s, Px, Py, tz + jb_t - bw if False else tz + jb_t - bw), 'buy', '608-2RS')
        A.add_body(c, 'BRG_P_IN_' + sn, brg(s, Px, Py, tz), 'buy', '608-2RS')
        z0 = zbl0 - 2.0
        A.add_body(c, 'BRG_B_IN_' + sn, brg(s, Bx, By, z0), 'buy', '608-2RS')
        A.add_body(c, 'BRG_B_OUT_' + sn, brg(s, Bx, By, z0 + jb_t - bw), 'buy', '608-2RS')

    _, c = A.fresh_component(asm, 'BUY_ROCKER_SHAFT')
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        A.add_body(c, 'ROCKER_SHAFT_' + sn, A.cylz(s * 28.0, s * (zr + lt / 2 + 4), Px, Py, P('rb_pin_d') / 2), 'steel', '8 mm shaft')
    _, c = A.fresh_component(asm, 'BUY_DIFF_HOUSING')
    hs = A.box(Px - 25, Px + 25, Py - 18, Py + 18, -28, 28)
    A.add_body(c, 'DIFF_HOUSING', hs, 'buy', 'bevel-gear differential placeholder [VERIFY]')
    _, c = A.fresh_component(asm, 'BUY_BOGIE_PIN')
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        A.add_body(c, 'BOGIE_PIN_' + sn, A.cylz(s * (zbl0 - 2.0), s * (zr + lt / 2 + 2), Bx, By, P('rb_pin_d') / 2), 'steel', '8 mm pin')

    seen = set()
    for cc in asm.occurrences:
        if cc.component.name not in seen:
            seen.add(cc.component.name); A.report(cc.component)
