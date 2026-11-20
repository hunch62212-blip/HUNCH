# 50_arm.py - both 3-DOF arms (starboard = ASM_ARM_R, s=+1; port = ASM_ARM_L, s=-1; each side is drawn, never mirrored).
# Fixed:  PRT_TOWER_x (deck base + shoulder servo cradle + pivot boss)        BUY_SHOULDER_SERVO_x
# Link 1: PRT_UPPER_ARM_x (twin plates, elbow cradle, 2 spacers)              BUY_ELBOW_SERVO_x, BUY_SHOULDER_HORN_x
# Link 2: PRT_FORE_ARM_x (twin plates, wrist bracket with MG996R pocket, ToF mount)   BUY_WRIST_SERVO_x, BUY_ELBOW_HORN_x
# Link 3 (hook + latch) is built by 60_hook_latch.py.   Servos are stylised: shaft axis = H (plan: gap = W, corrected).
# Everything is built in link frames and placed with link_mats() at the stow pose; 95_pose.py re-places the links.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def horn(s, d0, d1, x, y):
    h = A.cylz(s * d0, s * d1, x, y, A.P('arm_horn_d') / 2.0)
    return h

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    ar = A.arm_layers()
    S, L1, L2 = A.arm_geom()
    q = A.arm_pose_q('stow')
    mats, E, W = A.link_mats(q)
    ph, t, c = P('arm_plate_h'), P('arm_plate_t'), P('clr_slip')
    pin = P('arm_pin_d')
    bc = P('horn_bc') / 2.0
    for side, asm_name in ((1, 'ASM_ARM_R'), (-1, 'ASM_ARM_L')):
        s = side; tag = 'R' if s > 0 else 'L'
        asm = A.asm(asm_name)
        # clear this side if re-run
        for o in list(asm.occurrences):
            o.deleteMe()
        def bolt_ring(body, x, y, d0, d1):
            for k in range(int(P('horn_holes'))):
                a = math.pi / 2 * k
                A.cut(body, A.cylz(s * (d0 - 1), s * (d1 + 1), x + bc * math.cos(a), y + bc * math.sin(a), P('bolt_m3_clr') / 2))
        # ================= tower (fixed, world frame) =================================================
        _, comp = A.fresh_component(asm, 'PRT_TOWER_' + tag)
        shx, shy = S
        sol, pocket, ledge, pts = A.servo_cradle('big', shx, shy, ar['srv'][1], -1, s, -1, ar['wall'][0])
        xc = shx - (A.servo_spec('big')['L'] - A.servo_spec('big')['off'] - A.servo_spec('big')['off']) / 2.0
        deck = P('ch_deckY')
        post = A.box(xc - 44, xc + 44, deck, shy - P('srv_big_W') / 2 - 4, s * ar['wall'][0], s * ledge)
        A.union(sol, post)
        base = A.rbox_y(shx - 58 - 8, shx + 23 + 8, deck, deck + 5, s * (ar['wall'][0] + 3 - 8), s * (ar['srv'][1] - 3 + 8), 4)
        A.union(sol, base)
        for x in (shx - 58, shx + 23):
            for d in (ar['wall'][0] + 3, ar['srv'][1] - 3):
                A.cut(sol, A.hole((x, deck + 5, s * d), (0, -1, 0), P('bolt_m3_clr'), 5))
        A.cut(sol, pocket)
        A.union(sol, A.cylz(s * (ar['wall'][0] + 0.1), s * (ar['uin'][0] - 0.6), shx, shy, pin / 2))     # pivot pin boss
        A.cut(sol, A.insert_hole((shx, shy, s * (ar['uin'][0] - 0.6)), (0, 0, s), 'm3'))
        A.add_body(comp, 'TOWER_' + tag, sol, 'yellow', 'PETG 4 walls 40% infill; shoulder servo cradle')
        _, comp = A.fresh_component(asm, 'BUY_SHOULDER_SERVO_' + tag)
        sv, _i = A.servo_model('big', shx, shy, ar['srv'][1], -1, s, -1)
        A.add_body(comp, 'SERVO_SHOULDER_' + tag, sv, 'buy', '60 kg-class servo [VERIFY size]')

        # ================= link 1 =======================================================================
        _, comp = A.fresh_component(asm, 'PRT_UPPER_ARM_' + tag, mats[0])
        ex, ey = L1, 0.0
        sol_e, pocket_e, ledge_e, pts_e = A.servo_cradle('big', ex, ey, ar['srv'][1], -1, s, -1, ar['wall'][0])
        A.cut(sol_e, pocket_e)
        A.union(sol_e, A.cylz(s * (ar['wall'][0] + 0.1), s * (ar['fin'][1] - 0.5), ex, ey, pin / 2))         # elbow pin boss (F_in rides on it)
        A.cut(sol_e, A.insert_hole((ex, ey, s * (ar['fin'][1] - 0.5)), (0, 0, s), 'm3'))
        spx = [sx_ for sx_ in (ex - 24, ex - 24 + 0)]
        floor_pts = [(ex - 17.75 + dx, dy) for dx in (-30, 30) for dy in (-8, 8)]
        for (x, y) in floor_pts:
            A.cut(sol_e, A.insert_hole((x, y, s * ar['wall'][0]), (0, 0, s), 'm3'))
        A.add_body(comp, 'ELBOW_CRADLE_' + tag, sol_e, 'yellow', 'PETG')
        for nm, key in (('UPPER_PLATE_IN_', 'uin'), ('UPPER_PLATE_OUT_', 'uout')):
            d0, d1 = ar[key]
            pl = A.bar_xy((0, 0), (L1, 0), ph, s * d0, s * d1)
            A.union(pl, A.rbox_z(L1 - 62, L1 + 28, -ph / 2, ph / 2, s * d0, s * d1, 6))
            A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), 0, 0, (pin + 2 * c) / 2 if key == 'uin' else 5.0))
            if key == 'uin':
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), L1, 0, (pin + 2 * c) / 2))
                for (x, y) in floor_pts:
                    A.cut(pl, A.hole((x, y, s * d0), (0, 0, 1), P('bolt_m3_clr'), d1 - d0, over=1.0) if s > 0 else A.hole((x, y, s * d0), (0, 0, -1), P('bolt_m3_clr'), d1 - d0, over=1.0))
            else:
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), L1, 0, 5.5))
                bolt_ring(pl, 0, 0, d0, d1)                          # shoulder horn bolts
            for fx in (0.4 * L1, 0.62 * L1):                         # spacer screws
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), fx, 0, P('bolt_m3_clr') / 2))
            A.add_body(comp, nm + tag, pl, 'yellow', 'PETG 4 walls 40% infill')
        for k, fx in enumerate((0.4 * L1, 0.62 * L1)):
            sp = A.cylz(s * ar['uin'][1], s * ar['uout'][0], fx, 0, 5.0)
            A.cut(sp, A.insert_hole((fx, 0, s * ar['uin'][1]), (0, 0, s), 'm3'))
            A.cut(sp, A.insert_hole((fx, 0, s * ar['uout'][0]), (0, 0, -s), 'm3'))
            A.add_body(comp, 'ARM_SPACER_%d_%s' % (k + 1, tag), sp, 'yellow', 'PETG')
        _, comp = A.fresh_component(asm, 'BUY_ELBOW_SERVO_' + tag, mats[0])
        sv, _i = A.servo_model('big', ex, ey, ar['srv'][1], -1, s, -1)
        A.add_body(comp, 'SERVO_ELBOW_' + tag, sv, 'buy', '60 kg-class servo [VERIFY size]')
        _, comp = A.fresh_component(asm, 'BUY_SHOULDER_HORN_' + tag, mats[0])
        A.add_body(comp, 'HORN_SHOULDER_' + tag, horn(s, ar['horn'][0], ar['horn'][1], 0, 0), 'buy', '25T disc horn')

        # ================= link 2 =======================================================================
        _, comp = A.fresh_component(asm, 'PRT_FORE_ARM_' + tag, mats[1])
        wx = L2
        sv_spec = A.servo_spec('std')
        x_w0, x_w1 = wx - (sv_spec['L'] - sv_spec['off']), wx + sv_spec['off']
        xcw = (x_w0 + x_w1) / 2.0
        br_d0, br_d1 = ar['wsrv'][0], ar['fout'][0]
        brk = A.box(xcw - sv_spec['fl'] / 2 - 3, xcw + sv_spec['fl'] / 2 + 3, -ph / 2, ph / 2, s * br_d0, s * br_d1)
        _sol, pk, ledge_w, pts_w = A.servo_cradle('std', wx, 0.0, ar['wsrv'][0], 1, s, -1, br_d1)
        A.cut(brk, pk)
        # ear slot (widened pocket above the ledge) so the servo ears can sit on the ledge
        A.cut(brk, A.box(xcw - sv_spec['fl'] / 2 - c, xcw + sv_spec['fl'] / 2 + c, -sv_spec['W'] / 2 - c, sv_spec['W'] / 2 + c, s * (br_d0 - 1), s * ledge_w))
        for (x, y) in pts_w:
            A.cut(brk, A.insert_hole((x, y, s * ledge_w), (0, 0, s), 'm3'))
        bpts = [(xcw + dx, dy) for dx in (-20, 20) for dy in (-9, 9)]
        for (x, y) in bpts:
            A.cut(brk, A.insert_hole((x, y, s * br_d0), (0, 0, s), 'm3'))
            A.cut(brk, A.insert_hole((x, y, s * br_d1), (0, 0, -s), 'm3'))
        A.add_body(comp, 'WRIST_BRKT_' + tag, brk, 'yellow', 'PETG 5 walls (stiff)')
        for nm, key in (('FORE_PLATE_IN_', 'fin'), ('FORE_PLATE_OUT_', 'fout')):
            d0, d1 = ar[key]
            pl = A.bar_xy((0, 0), (L2, 0), ph, s * d0, s * d1)
            if key == 'fin':
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), 0, 0, (pin + 2 * c) / 2))
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), wx, 0, 7.0))               # wrist shaft / horn
            else:
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), 0, 0, 5.5))
                bolt_ring(pl, 0, 0, d0, d1)                                              # elbow horn bolts
                A.union(pl, A.rbox_z(L2 - 40, L2 + 14, -ph / 2, ph / 2, s * d0, s * d1, 6))
            for (x, y) in bpts:
                A.cut(pl, A.cylz(s * (d0 - 1), s * (d1 + 1), x, y, P('bolt_m3_clr') / 2))
            A.add_body(comp, nm + tag, pl, 'yellow', 'PETG 4 walls 40% infill')
        tof = A.box(L2 - 85, L2 - 62, -ph / 2 - 8, -ph / 2 + 1, s * (ar['fin'][0] - 3), s * ar['fin'][1])
        for dx in (-7.5, 7.5):
            A.cut(tof, A.insert_hole((L2 - 73.5 + dx, -ph / 2 - 8, s * (ar['fin'][0] + 1.5)), (0, 1, 0), 'm25'))
        A.add_body(comp, 'TOF_MOUNT_' + tag, tof, 'yellow', 'PETG; VL53L1X board')
        _, comp = A.fresh_component(asm, 'BUY_WRIST_SERVO_' + tag, mats[1])
        sv, _i = A.servo_model('std', wx, 0.0, ar['wsrv'][0], 1, s, -1)
        A.add_body(comp, 'SERVO_WRIST_' + tag, sv, 'buy', 'MG996R [VERIFY]')
        _, comp = A.fresh_component(asm, 'BUY_ELBOW_HORN_' + tag, mats[1])
        A.add_body(comp, 'HORN_ELBOW_' + tag, horn(s, ar['horn'][0], ar['horn'][1], 0, 0), 'buy', '25T disc horn')
        _, comp = A.fresh_component(asm, 'BUY_WRIST_HORN_' + tag, mats[2])
        A.add_body(comp, 'HORN_WRIST_' + tag, horn(s, ar['whorn'][0], ar['whorn'][1], 0, 0), 'buy', '25T disc horn')
    for sd in ('ASM_ARM_R', 'ASM_ARM_L'):
        for o in A.asm(sd).occurrences:
            A.report(o.component)
