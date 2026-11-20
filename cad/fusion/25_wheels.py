# 25_wheels.py - ASM_WHEELS: WHEEL_DRIVEN (x4: FL FR RL RR) and WHEEL_IDLER (x2: ML MR), 130 x 75 mm sand wheels.
# Wheel frame: origin = wheel centre, +z outboard.  Hoop (BUY, 3 mm woven-steel stand-in) with 20 chevron cleats,
# printed PETG disc at the outer face (lightening holes; 4 x M3 on whub_bc for the Pololu hub, or on idl_bc for the
# idler hub), BUY_HUB_6MM on the driven wheels.  Left wheels are the right wheels rotated 180 deg about the vertical.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def wheel_mat(w):
    return A.mat(w['x'], w['y'], w['z'], ry=0.0 if w['side'] > 0 else 180.0)

def tyre():
    P = A.P
    R, h, rim = P('wh_r'), P('wh_cleat_h'), P('wh_rim_t')
    half = P('wh_w') / 2
    ro = R - h
    body = A.tube_z(-half, half, 0, 0, ro, ro - rim)
    n = int(P('wh_cleats'))
    # chevron pair at angle 0 (radial = +x): two 39 mm half-cleats tilted +-25 deg about the radial axis
    cw = 4.0
    a = A.box(ro - 3.0, R - 2.6, -cw / 2, cw / 2, 0, 39.0)
    A.rot_about(a, 'x', 25.0, (0, 0, 0))
    b = A.box(ro - 3.0, R - 2.6, -cw / 2, cw / 2, -39.0, 0)
    A.rot_about(b, 'x', -25.0, (0, 0, 0))
    A.union(a, b)
    for k in range(n):
        A.union(body, A.rot_about(A.copy(a), 'z', 360.0 * k / n, (0, 0, 0)))
    return body

def disc(driven):
    P = A.P
    tz = P('tr_z')
    z1 = P('wh_outer_z') - tz; z0 = z1 - P('wh_disc_t')
    r = P('wh_r') - P('wh_cleat_h') - P('wh_rim_t') - P('clr_snug')
    d = A.cylz(z0, z1, 0, 0, r)
    if driven:
        bc, ang0, rl = P('whub_bc'), 0.0, 36.0
        A.cut(d, A.hole((0, 0, z1), (0, 0, -1), P('mot_shaft_d') + 1.0, P('wh_disc_t')))
        n = int(P('horn_holes'))
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            A.cut(d, A.hole((bc / 2 * math.cos(a), bc / 2 * math.sin(a), z1), (0, 0, -1), P('bolt_m3_clr'), P('wh_disc_t')))
    else:
        bc, rl = P('idl_bc'), 38.0
        A.cut(d, A.hole((0, 0, z1), (0, 0, -1), P('rb_pin_d') + 1.0, P('wh_disc_t')))
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            A.cut(d, A.hole((bc / 2 * math.cos(a), bc / 2 * math.sin(a), z1), (0, 0, -1), P('bolt_m3_clr'), P('wh_disc_t')))
    for k in range(6):
        a = k * math.pi / 3 + (math.pi / 6 if not driven else 0.0)
        A.cut(d, A.hole((rl * math.cos(a), rl * math.sin(a), z1), (0, 0, -1), 14.0, P('wh_disc_t')))
    return d

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_WHEELS')
    W = A.wheels()
    tz = P('tr_z')
    z1 = P('wh_outer_z') - tz
    z0 = z1 - P('wh_disc_t')

    drv = ['FL', 'FR', 'RL', 'RR']
    occ, c = A.fresh_component(asm, 'WHEEL_DRIVEN', wheel_mat(W['FL']))
    A.add_body(c, 'TYRE_HOOP', tyre(), 'rubber', 'woven-steel hoop + cleats')
    A.add_body(c, 'PRT_WHEEL_DISC', disc(True), 'darkgrey', 'PETG')
    hub = A.cylz(z0 - P('whub_L'), z0, 0, 0, P('whub_d') / 2)
    A.cut(hub, A.hole((0, 0, z0 - P('whub_L')), (0, 0, 1), P('mot_shaft_d'), P('whub_L') + 1))
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        A.cut(hub, A.hole((P('whub_bc') / 2 * math.cos(a), P('whub_bc') / 2 * math.sin(a), z0), (0, 0, -1), 2.5, 8.0))
    A.add_body(c, 'BUY_HUB_6MM', hub, 'alu', 'Pololu 6 mm universal hub')
    for i, k in enumerate(drv):
        if i:
            A.place(asm, c, wheel_mat(W[k]))
    idl = ['ML', 'MR']
    occ, c = A.fresh_component(asm, 'WHEEL_IDLER', wheel_mat(W['ML']))
    A.add_body(c, 'TYRE_HOOP', tyre(), 'rubber', 'woven-steel hoop + cleats')
    A.add_body(c, 'PRT_WHEEL_DISC_IDLER', disc(False), 'grey', 'PETG')
    A.place(asm, c, wheel_mat(W['MR']))
    A.report(c)
    print('wheel occurrences:', len(asm.occurrences))
