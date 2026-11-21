# 40_power_electronics.py - ASM_POWER (Option R weight layout, plan section 3): battery LOW at x = bat_x (-80 mm), electronics
# at x = ele_x (+50 mm).
#   PRT_BATTERY_TRAY   pocket bat_L/W + 2*clr_slip, 3 mm walls, 5 mm floor, 4 strap slots, 4 M3 floor bolts, wire notch,
#                      4 corner posts with M3 inserts for the lid
#   PRT_BATTERY_LID    ASA lid (3 mm) on the posts
#   BUY_BATTERY        LiFePO4 12.8 V 10 Ah envelope bat_L x bat_W x bat_H at bat_y (bottom)
#   PRT_ELEC_TRAY_*    lower plate (2 halves, split at z = 0 by 2 tongue-and-groove keys + M3 bolts / inserts) with M2.5
#                      heat-set standoffs for the 2 x MDD10A, 4 tall posts carrying the upper plate
#   PRT_ELEC_UPPER_*   upper plate (2 halves) with M2.5 standoffs for Pi 5, ESP32-S3, PCA9685, 2 x UBEC, 2 x INA226, IMU
#   BUY_BOARDS         placeholder boards sized from params (translucent blue)
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_POWER')
    floor_y = P('ch_tubY0')
    slip = P('clr_slip')
    bx, bL, bW, bH, by = P('bat_x'), P('bat_L'), P('bat_W'), P('bat_H'), P('bat_y')
    tw, tf, th = P('bat_tray_wall'), P('bat_tray_floor'), P('bat_tray_wallH')

    # ================= battery tray ================================================================
    px, pz = bL / 2 + slip, bW / 2 + slip                 # pocket half sizes
    ox, oz = px + tw, pz + tw                             # outer half sizes of the walls
    _, c = A.fresh_component(asm, 'PRT_BATTERY_TRAY')
    fx, fz = bL / 2 + 8 + 6.0, 61.2                       # floor flange reaches the 4 floor bolts
    tray = A.rbox_y(bx - fx, bx + fx, floor_y, floor_y + tf, -fz, fz, 5.0)
    wall = A.rbox_y(bx - ox, bx + ox, floor_y + tf, floor_y + tf + th, -oz, oz, 2.0)
    A.cut(wall, A.box(bx - px, bx + px, floor_y, floor_y + tf + th + 1, -pz, pz))
    A.union(tray, wall)
    A.cut(tray, A.box(bx + px - 1, bx + ox + 1, floor_y + tf, floor_y + tf + 15, -10, 10))      # wire notch (+x wall)
    for sx in (-1, 1):
        for sz in (-1, 1):
            A.cut(tray, A.hole((bx + sx * (bL / 2 + 8), floor_y + tf, sz * 58.5), (0, -1, 0), P('bolt_m3_clr'), tf))
        for sz in (-1, 1):                                # strap slots 20 x 3 beside the walls
            A.cut(tray, A.box(bx + sx * 35 - 10, bx + sx * 35 + 10, floor_y - 1, floor_y + tf + 1, sz * 57.2 - 1.5, sz * 57.2 + 1.5))
    lid_y = by + bH + 0.2
    boss = P('ins_m3_boss')
    for sx in (-1, 1):
        for sz in (-1, 1):
            q = (bx + sx * ox, sz * oz)
            A.union(tray, A.cyly(floor_y + tf, lid_y, q[0], q[1], boss / 2))
            A.cut(tray, A.insert_hole((q[0], lid_y, q[1]), (0, -1, 0), 'm3'))
    A.add_body(c, 'BATTERY_TRAY', tray, 'petg', 'PETG 4 walls, flat, 35% infill')

    _, c = A.fresh_component(asm, 'PRT_BATTERY_LID')
    lt = P('bat_lid_t')
    lid = A.rbox_y(bx - ox - boss / 2, bx + ox + boss / 2, lid_y, lid_y + lt, -oz - boss / 2, oz + boss / 2, 4.0)
    for sx in (-1, 1):
        for sz in (-1, 1):
            A.cut(lid, A.hole((bx + sx * ox, lid_y + lt, sz * oz), (0, -1, 0), P('bolt_m3_clr'), lt))
    for k in range(5):                                   # vent slots
        A.cut(lid, A.box(bx - 50 + 25 * k - 6, bx - 50 + 25 * k + 6, lid_y - 1, lid_y + lt + 1, -30, 30))
    A.add_body(c, 'BATTERY_LID', lid, 'asa', 'ASA 3 walls')

    _, c = A.fresh_component(asm, 'BUY_BATTERY')
    bat = A.box(bx - bL / 2, bx + bL / 2, by, by + bH, -bW / 2, bW / 2)
    for sz in (-15, 15):
        A.union(bat, A.cylx(bx + bL / 2, bx + bL / 2 + 6, by + 70, sz, 4.0))              # terminals toward +x
    A.add_body(c, 'BATTERY_LIFEPO4', bat, 'buy', 'LiFePO4 12.8 V 10 Ah + BMS [VERIFY size]')

    # ================= electronics trays ===============================================================
    ex, eL = P('ele_x'), P('ele_L')
    x0, x1 = ex - eL / 2, ex + eL / 2
    pt = P('ele_plate_t'); sh = P('ele_standoff_h')
    up_y = P('ele_upper_y')
    pad_h = 10.0
    key_xs = (ex - 22.0, ex + 22.0)
    zhalf = 143.0
    bo = P('ele_hole_inset')

    # board table: key -> (plate, side, x_min, z_near, w(x), l(z), h, holes[(dx, dz)] or None, body name)
    def gen_holes(w, l):
        return [(bo, bo), (w - bo, bo), (bo, l - bo), (w - bo, l - bo)]
    pi_h = [(bo, bo), (bo + P('pi_hdy'), bo), (bo, bo + P('pi_hdx')), (bo + P('pi_hdy'), bo + P('pi_hdx'))]
    boards = [
        ('lower', +1, 12.0, 28.0, P('mdd_W'), P('mdd_L'), P('mdd_H'), gen_holes(P('mdd_W'), P('mdd_L')), 'MDD10A_A'),
        ('lower', -1, 12.0, 28.0, P('mdd_W'), P('mdd_L'), P('mdd_H'), gen_holes(P('mdd_W'), P('mdd_L')), 'MDD10A_B'),
        ('upper', +1, 8.0, 28.0, P('pi_W'), P('pi_L'), P('pi_H'), pi_h, 'RASPBERRY_PI5'),
        ('upper', +1, 68.0, 28.0, P('esp_W'), P('esp_L'), P('esp_H'), gen_holes(P('esp_W'), P('esp_L')), 'ESP32_S3'),
        ('upper', +1, 70.0, 102.0, P('imu_W'), P('imu_L'), P('imu_H'), gen_holes(P('imu_W'), P('imu_L')), 'BNO085_IMU'),
        ('upper', -1, 8.0, 28.0, P('pca_W'), P('pca_L'), P('pca_H'), gen_holes(P('pca_W'), P('pca_L')), 'PCA9685'),
        ('upper', -1, 38.0, 28.0, P('ubec_W'), P('ubec_L'), P('ubec_H'), gen_holes(P('ubec_W'), P('ubec_L')), 'UBEC_A_7V4'),
        ('upper', -1, 38.0, 82.0, P('ubec_W'), P('ubec_L'), P('ubec_H'), gen_holes(P('ubec_W'), P('ubec_L')), 'UBEC_B_6V0'),
        ('upper', -1, 70.0, 28.0, P('ina_W'), P('ina_L'), P('ina_H'), gen_holes(P('ina_W'), P('ina_L')), 'INA226_A'),
        ('upper', -1, 70.0, 58.0, P('ina_W'), P('ina_L'), P('ina_H'), gen_holes(P('ina_W'), P('ina_L')), 'INA226_B'),
    ]
    post_pts = [(6.0, 40.0), (6.0, 120.0), (94.0, 40.0), (94.0, 120.0)]    # (x, |z|) of the upper-plate posts

    def build_plate(y0, pad_up=True):
        pl = A.box(x0, x1, y0, y0 + pt, -zhalf, zhalf)
        A.union(pl, A.box(ex - 31, ex + 31, y0 + pt, y0 + pt + pad_h - 0.0, -24, 24))
        return pl

    def standoffs(body, plate_top, side, plate):
        n = 0
        for (pl, sd, bx0, bz0, w, l, h, holes, nm) in boards:
            if pl != plate or sd != side:
                continue
            for (dx, dz) in holes:
                q = (x0 + bx0 + dx, side * (bz0 + dz))
                A.union(body, A.cyly(plate_top, plate_top + sh, q[0], q[1], (P('ins_m25_d') + 2 * P('wall_min')) / 2))
                A.cut(body, A.insert_hole((q[0], plate_top + sh, q[1]), (0, -1, 0), 'm25'))
                n += 1
        return n

    # ---- lower plate ---------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'PRT_ELEC_TRAY')
    pad_mid = floor_y + pt + pad_h / 2
    lower = build_plate(floor_y)
    for (x, z) in A.floor_bolts():
        if x0 - 1 < x < x1 + 1:
            A.cut(lower, A.hole((x, floor_y + pt, z), (0, -1, 0), P('bolt_m3_clr'), pt, over=0.5))
    lo, hi = A.split_keyed(lower, 'z', 0.0, [(kx, pad_mid) for kx in key_xs], key_w=14.0, key_h=5.0, key_depth=14.0)
    for body, side, nm in ((hi, +1, 'ELEC_TRAY_STBD'), (lo, -1, 'ELEC_TRAY_PORT')):
        standoffs(body, floor_y + pt, side, 'lower')
        for (px_, pz_) in post_pts:                          # tall posts for the upper plate
            A.union(body, A.cyly(floor_y + pt, up_y, px_ + 0.0 + 0.0, side * pz_, 5.0))
            A.cut(body, A.insert_hole((px_, up_y, side * pz_), (0, -1, 0), 'm3'))
        A.add_body(c, nm, body, 'petg', 'PETG 4 walls, flat')

    # ---- upper plate ---------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'PRT_ELEC_UPPER')
    upper = build_plate(up_y)
    for (px_, pz_) in post_pts:
        for sd in (1, -1):
            A.cut(upper, A.hole((px_, up_y + pt, sd * pz_), (0, -1, 0), P('bolt_m3_clr'), pt, over=0.5))
    pad_mid_u = up_y + pt + pad_h / 2
    lo, hi = A.split_keyed(upper, 'z', 0.0, [(kx, pad_mid_u) for kx in key_xs], key_w=14.0, key_h=5.0, key_depth=14.0)
    for body, side, nm in ((hi, +1, 'ELEC_UPPER_STBD'), (lo, -1, 'ELEC_UPPER_PORT')):
        standoffs(body, up_y + pt, side, 'upper')
        A.add_body(c, nm, body, 'petg', 'PETG 4 walls, flat')

    # ---- placeholder boards -----------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'BUY_BOARDS')
    for (pl, sd, bx0, bz0, w, l, h, holes, nm) in boards:
        yb = (floor_y + pt + sh) if pl == 'lower' else (up_y + pt + sh)
        zs = sorted((sd * bz0, sd * (bz0 + l)))
        A.add_body(c, nm, A.box(x0 + bx0, x0 + bx0 + w, yb, yb + h, zs[0], zs[1]), 'buy', 'placeholder [VERIFY size]')

    # Pi 5 Active Cooler placeholder on top of the Pi (about 40 x 40 x 9 over the SoC; the cooler fan inlet needs pi_cool_clear
    # of free air above it, checked in tests/cad_params.mjs against the deck underside) [VERIFY size]
    for (pl, sd, bx0, bz0, w, l, h, holes, nm) in boards:
        if nm == 'RASPBERRY_PI5':
            yb = up_y + pt + sh + h
            zc = sd * (bz0 + l / 2)
            A.add_body(c, 'BUY_PI5_ACTIVE_COOLER', A.box(x0 + bx0 + w / 2 - 20, x0 + bx0 + w / 2 + 20, yb, yb + 9.0, zc - 20, zc + 20),
                       'buy', 'Raspberry Pi 5 Active Cooler placeholder [VERIFY size]; keep pi_cool_clear free above it')

    seen = set()
    for cc in asm.occurrences:
        if cc.component.name not in seen:
            seen.add(cc.component.name); A.report(cc.component)
