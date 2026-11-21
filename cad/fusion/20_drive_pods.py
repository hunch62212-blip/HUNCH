# 20_drive_pods.py - ASM_DRIVE: four driven pods (FL, FR, RL, RR) and two passive idler pods (ML, MR).
# Pod frame: origin = wheel centre, +z outboard, local +x = arm toward the link pivot (rocker P for F*, bogie B for M*/R*).
# Components (each instanced with A.pod_mat, never mirrored):
#   PRT_MOTOR_MOUNT (orange, printed in ASA)  clamp ring (mot_d + 2 clr_slip) with pinch slit + M3 insert/clearance, 12 mm flange arm with
#                             2 x M4 heat-set inserts (bolts pass through the link bar), face ring with 6 x M3 on mot_bc_d,
#                             two struts, wire notch.
#   BUY_MOTOR_37D             Pololu 37D + encoder (68 mm), D-shaft, collar, 6 face holes.
#   PRT_IDLER_BOSS / BUY_AXLE_8MM / PRT_IDLER_HUB (grey) / BUY_608   passive idler: fixed 8 mm axle in the boss,
#                             hub on two 608 press seats with crush ribs, disc bolts to 4 x M3 inserts on idl_bc.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_DRIVE')
    W = A.wheels()
    tz = P('tr_z')
    zl = P('rb_face_z') - tz                 # link face (local)     -54
    zb = P('mot_z0') - tz                    # motor back (local)    -51.5
    zf = P('mot_z_face') - tz                # motor face            16.5
    md = P('mot_d'); slip = P('clr_slip')
    r_in = md / 2 + slip; r_out = r_in + P('mot_clamp_wall')
    ft = P('mot_flange_t'); ring_t = P('mot_ring_t')
    off, pitch = P('mot_ear_off'), P('mot_ear_pitch')
    arm_w = P('rb_link_h')

    # ================= BUY_MOTOR_37D ==================================================================
    occ, c = A.fresh_component(asm, 'BUY_MOTOR_37D', A.pod_mat(W['FL']))
    enc = A.cylz(zb, zb + 8, 0, 0, md / 2)
    A.add_body(c, 'ENCODER', enc, 'black')
    can = A.cylz(zb + 8, zf - 30, 0, 0, md / 2)
    A.add_body(c, 'MOTOR_CAN', can, 'steel')
    gb = A.cylz(zf - 30, zf, 0, 0, md / 2)
    for k in range(int(P('mot_holes'))):
        a = 2 * math.pi * k / P('mot_holes')
        A.cut(gb, A.hole((P('mot_bc_d') / 2 * math.cos(a), P('mot_bc_d') / 2 * math.sin(a), zf), (0, 0, -1), 3.0, 6.0))
    A.add_body(c, 'GEARBOX', gb, 'alu')
    sh = A.cylz(zf, zf + P('mot_shaft_L'), 0, 0, P('mot_shaft_d') / 2)
    A.cut(sh, A.box(-5, 5, 2.5, 5, zf + P('mot_shaft_L') - 12, zf + P('mot_shaft_L') + 1))     # D flat
    A.union(sh, A.cylz(zf, zf + ring_t, 0, 0, P('mot_collar_d') / 2))
    A.add_body(c, 'SHAFT_COLLAR', sh, 'steel')
    for b in c.bRepBodies:
        b.appearance = A.appearance('buy') if b.name in ('ENCODER', 'MOTOR_CAN') else b.appearance
    A.place_pods(asm, occ, c, 'BUY_MOTOR_37D', ['FL', 'FR', 'RL', 'RR'], W)

    # ================= PRT_MOTOR_MOUNT =================================================================
    occ, c = A.fresh_component(asm, 'PRT_MOTOR_MOUNT', A.pod_mat(W['FL']))
    z_ring_end = zl + P('mot_clamp_L')
    body = A.cylz(zl, z_ring_end, 0, 0, r_out)
    # flange arm along +x (stadium), thickness ft, from the axle to the second M4 boss
    arm_end = off + pitch
    A.union(body, A.bar_xy((0, 0), (arm_end, 0), arm_w, zl, zl + ft))
    A.union(body, A.cylz(zl, zl + ft, 0, 0, r_out))
    # pinch ears on top (+y) and the pinch bolt (along x): clearance side (-x), insert side (+x)
    ear_z0, ear_z1 = zl + 8, zl + 24
    A.union(body, A.box(-10, 10, r_out - 4, r_out + 9, ear_z0, ear_z1))
    # face ring and two side struts
    A.union(body, A.cylz(zf, zf + ring_t, 0, 0, r_out))
    for s in (1, -1):
        A.union(body, A.box(s * (r_in + 0.8), s * (r_in + 6.8), -3, 3, zl + ft - 1, zf + ring_t))
    # bore for the motor (through the whole clamp ring + flange), clamp slit, wire notch
    A.cut(body, A.cylz(zl - 1, z_ring_end, 0, 0, r_in))
    A.cut(body, A.cylz(zf - 1, zf + ring_t + 1, 0, 0, r_in - 1.0 if False else P('mot_collar_d') / 2 + slip))
    A.cut(body, A.box(-P('mot_pinch_gap') / 2, P('mot_pinch_gap') / 2, 0, r_out + 12, zl - 1, z_ring_end + 1))
    A.cut(body, A.box(-7, 7, -r_out - 1, -r_in + 3, zl - 1, zl + 6))                       # encoder wire notch
    yb = r_out + 3.5; zb_ = (ear_z0 + ear_z1) / 2
    A.cut(body, A.hole((-10, yb, zb_), (1, 0, 0), P('bolt_m3_clr'), 10.0))
    A.cut(body, A.insert_hole((P('mot_pinch_gap') / 2, yb, zb_), (1, 0, 0), 'm3'))
    # face ring: six M3 clearance holes on mot_bc_d
    for k in range(int(P('mot_holes'))):
        a = 2 * math.pi * k / P('mot_holes')
        A.cut(body, A.hole((P('mot_bc_d') / 2 * math.cos(a), P('mot_bc_d') / 2 * math.sin(a), zf + ring_t), (0, 0, -1), P('bolt_m3_clr'), ring_t + 0.5))
    # two M4 heat-set inserts from the link face (flange thick enough: insert + 1 mm + >= 2 mm wall)
    for k in range(2):
        x = off + k * pitch
        A.union(body, A.cylz(zl, zl + ft, x, 0, P('ins_m4_boss') / 2))
        A.cut(body, A.insert_hole((x, 0, zl), (0, 0, 1), 'm4'))
    A.add_body(c, 'MOTOR_MOUNT', body, 'orange', 'ASA 5 walls, load plane parallel to bed (motor can runs 70-90 C; PETG creeps above ~70 C); PA-CF optional')
    A.place_pods(asm, occ, c, 'PRT_MOTOR_MOUNT', ['FL', 'FR', 'RL', 'RR'], W)

    # ================= idler pod =========================================================================
    hub_z0, hub_z1 = -35.0, P('wh_outer_z') - tz - P('wh_disc_t')      # -35 .. 32.5
    bw = P('brg_w')
    occ, c = A.fresh_component(asm, 'PRT_IDLER_HUB', A.pod_mat(W['ML']))
    hub = A.cylz(hub_z0, hub_z1 - 4.5, 0, 0, P('idl_hub_d') / 2)
    A.union(hub, A.cylz(hub_z1 - 4.5, hub_z1, 0, 0, P('idl_flange_d') / 2))
    A.bearing_seat_cut(hub, (0, 0, hub_z0), (0, 0, 1))
    A.bearing_seat_cut(hub, (0, 0, hub_z1), (0, 0, -1))
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        A.cut(hub, A.insert_hole((P('idl_bc') / 2 * math.cos(a), P('idl_bc') / 2 * math.sin(a), hub_z1), (0, 0, -1), 'm3'))
    A.add_body(c, 'IDLER_HUB', hub, 'grey', 'PETG 5 walls')
    A.place_pods(asm, occ, c, 'PRT_IDLER_HUB', ['ML', 'MR'], W)

    occ, c = A.fresh_component(asm, 'BUY_608', A.pod_mat(W['ML']))
    for nm, z0 in (('BRG_INBOARD', hub_z0), ('BRG_OUTBOARD', hub_z1 - bw)):
        b = A.tube_z(z0, z0 + bw, 0, 0, P('brg_od') / 2, P('brg_id') / 2)
        A.add_body(c, nm, b, 'buy', '608-2RS')
    A.place_pods(asm, occ, c, 'BUY_608', ['ML', 'MR'], W)

    occ, c = A.fresh_component(asm, 'PRT_IDLER_BOSS', A.pod_mat(W['ML']))
    boss = A.bar_xy((0, 0), (arm_end, 0), arm_w, zl, zl + ft)
    A.union(boss, A.cylz(zl, zl + ft, 0, 0, P('idl_boss_d') / 2 + 8))
    A.union(boss, A.cylz(zl + ft, hub_z0 - 1.0, 0, 0, P('idl_boss_d') / 2))
    A.cut(boss, A.cylz(zl - 1, hub_z0, 0, 0, P('rb_pin_d') / 2 + P('clr_snug')))
    for k in range(2):
        x = off + k * pitch
        A.union(boss, A.cylz(zl, zl + ft, x, 0, P('ins_m4_boss') / 2))
        A.cut(boss, A.insert_hole((x, 0, zl), (0, 0, 1), 'm4'))
    A.add_body(c, 'IDLER_BOSS', boss, 'grey', 'PETG 5 walls')
    A.place_pods(asm, occ, c, 'PRT_IDLER_BOSS', ['ML', 'MR'], W)

    occ, c = A.fresh_component(asm, 'BUY_AXLE_8MM', A.pod_mat(W['ML']))
    A.add_body(c, 'AXLE_8MM', A.cylz(zl + 1, hub_z1 - 1.5, 0, 0, P('rb_pin_d') / 2), 'steel', 'M8 bolt / 8 mm shaft')
    A.place_pods(asm, occ, c, 'BUY_AXLE_8MM', ['ML', 'MR'], W)

    seen = set()
    for cc in asm.occurrences:
        if cc.component.name not in seen:
            seen.add(cc.component.name)
            A.report(cc.component)
    print('occurrences:', [o.name for o in asm.occurrences])
