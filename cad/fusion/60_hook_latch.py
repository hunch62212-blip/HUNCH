# 60_hook_latch.py - link 3 of both arms: PRT_HOOK_x (V-seat trunnion hook, mouth up, 30 deg lead-in cheeks, seat-switch
# pocket, MG90S latch cage + pocket, over-centre stop pin), PRT_LATCH_GATE_x (swings across the throat, 1 mm over the
# trunnion), BUY_LATCH_SERVO_x.  Frame: origin = wrist axis, +x along the forearm.  The V seat sits hook_off ahead of the wrist.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    ar = A.arm_layers()
    g = A.hook_geom()
    ho, yA, rt = g['ho'], g['yA'], g['rt']
    tanc = g['tan_c']; th = g['throat']; yt = g['y_throat']
    mats, E, W = A.link_mats(A.arm_pose_q('stow'))
    px, py = g['pivot']
    hook_d = ar['hook']; gate_d = ar['gate']
    c = P('clr_slip')
    bc = P('horn_bc') / 2.0
    ytop = 24.0
    for side, asm_name in ((1, 'ASM_ARM_R'), (-1, 'ASM_ARM_L')):
        s = side; tag = 'R' if s > 0 else 'L'
        asm = A.asm(asm_name)
        _, comp = A.fresh_component(asm, 'PRT_HOOK_' + tag, mats[2])
        z0, z1 = s * hook_d[0], s * hook_d[1]
        # plate: block + horn boss
        hk = A.rbox_z(-15, ho + 48, -24, ytop, z0, z1, 8.0)
        A.union(hk, A.cylz(z0, z1, 0, 0, 16.0))
        # V-seat + cheeks cut (convex pieces): V wedge, then the flared lead-in cheeks
        v = A.poly_prism_z([(ho, yA), (ho + th, yt), (ho - th, yt)][::-1] if False else [(ho, yA), (ho + th + 6, yt + 6), (ho - th - 6, yt + 6)][::1], min(z0, z1) - 1, max(z0, z1) + 1)
        # wedge above must be CCW for poly_prism_z: build explicitly
        v = A.poly_prism_z([(ho, yA), (ho + th, yt), (ho - th, yt)] if False else [(ho, yA), (ho + th, yt), (ho - th, yt)], min(z0, z1) - 1, max(z0, z1) + 1)
        A.cut(hk, v)
        ych = ytop + 6
        cheeks = A.poly_prism_z([(ho - th, yt), (ho + th, yt), (ho + th + (ych - yt) * tanc, ych), (ho - th - (ych - yt) * tanc, ych)], min(z0, z1) - 1, max(z0, z1) + 1)
        A.cut(hk, cheeks)
        # horn bolt circle + shaft hole
        A.cut(hk, A.cylz(min(z0, z1) - 1, max(z0, z1) + 1, 0, 0, 4.2))
        for k in range(int(P('horn_holes'))):
            a = math.pi / 2 * k
            A.cut(hk, A.cylz(min(z0, z1) - 1, max(z0, z1) + 1, bc * math.cos(a), bc * math.sin(a), P('bolt_m3_clr') / 2))
        # latch servo: shaft at the pivot, body toward +x, cap between cage walls, bulk in the housing
        sol, pocket, ledge, pts = A.servo_cradle('mic', px, py, ar['hsrv'][0], 1, s, 1, ar['hsrv'][1] + 8.0)
        sp = A.servo_spec('mic')
        A.union(hk, sol)
        A.cut(hk, pocket)
        xc = px + (sp['L'] - 2 * sp['off']) / 2.0
        for ysd in (-1, 1):                                   # cage walls from the plate to the housing
            A.union(hk, A.box(xc - sp['fl'] / 2 - 3, xc + sp['fl'] / 2 + 3, py + ysd * (sp['W'] / 2 + c) + (0 if ysd > 0 else -3), py + ysd * (sp['W'] / 2 + c) + (3 if ysd > 0 else 0), z1, s * ledge))
        A.cut(hk, A.cylz(min(z0, z1) - 1, max(z0, z1) + 1, px, py, 5.4))     # gate boss passes the plate
        # stop pin for the closed gate (over-centre rest), seat-switch pocket under the apex
        A.union(hk, A.cylz(z0 + s * 0.1, s * (gate_d[0] - 2.0 if s > 0 else gate_d[0] - 2.0), ho - th - 6 - 2.5, py, 2.5))
        A.cut(hk, A.box(ho - 6.5, ho + 6.5, yA - 15.0, yA - 8.5, z1 + s * 1.0, z1 - s * 4.0))
        A.cut(hk, A.insert_hole((ho, yA - 11.5, z1), (0, 0, -s), 'm25'))
        A.add_body(comp, 'HOOK_' + tag, hk, 'yellow', 'PETG 5 walls, 60% infill; print flat')
        # ---- gate (pivot frame, closed pose) ------------------------------------------------------------
        glen = (px - (ho - th - 4.0))
        _, comp = A.fresh_component(asm, 'PRT_LATCH_GATE_' + tag, mats[2])
        gt = P('gate_t')
        gz0, gz1 = s * gate_d[0], s * gate_d[1]
        gate = A.box(px - glen, px + 6, py - gt / 2, py + gt / 2, gz0, gz1)
        A.union(gate, A.cylz(gz0, gz1, px, py, 8.0))
        A.union(gate, A.cylz(s * gate_d[1], s * (hook_d[1] + 0.1), px, py, 5.0))           # boss through the plate
        A.cut(gate, A.cylz(s * (hook_d[1] - 3.0), s * (hook_d[1] + 0.2), px, py, 1.6))
        A.union(gate, A.cylz(gz0, gz1, px - glen, py, gt / 2))
        A.add_body(comp, 'LATCH_GATE_' + tag, gate, 'red', 'PETG; over-centre stop on the hook')
        _, comp = A.fresh_component(asm, 'BUY_LATCH_SERVO_' + tag, mats[2])
        sv, _i = A.servo_model('mic', px, py, ar['hsrv'][0], 1, s, 1)
        A.add_body(comp, 'SERVO_LATCH_' + tag, sv, 'buy', 'MG90S [VERIFY]')
        for nm in ('PRT_HOOK_', 'PRT_LATCH_GATE_'):
            A.report(A.find_occ(asm, nm + tag).component)
