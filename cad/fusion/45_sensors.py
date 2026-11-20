# 45_sensors.py - ASM_SENSORS: front LiDAR cradle (low, inside the 610 mm footprint, hung from the front cross rail),
# 2 x OV5647 cameras, rear LiDAR mast (top at sen_mast_top = 385), and the REF_SWEEP keep-out (arm swept volume).
#   PRT_SENSOR_NOSE (ASA): 8 mm platform + 2 hanger plates; LiDAR on 4 x M2.5 inserts, cameras on 4 x M2.5 inserts each,
#                          2 x M5 into the front rail T-nuts.
#   PRT_LIDAR_MAST_REAR (ASA): base plate bolted through the deck into the rear cross rail (2 x M5), column, ribs, 8 mm
#                          platform with 4 x M2.5 inserts.
# Then prints whether any body (other than REF_SWEEP / ASM_ARM) intrudes into the keep-out.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_SENSORS')
    rail_y0 = P('ch_rail_y1') - P('ext_size')
    cx = P('ch_cross_x')
    ld, lh = P('lidar_d'), P('lidar_h')
    ins = lambda: None

    # ================= front nose =======================================================================
    lx, ly = P('sen_nose_x'), P('sen_nose_y')
    plat_y1 = ly - lh / 2; plat_y0 = plat_y1 - 8.0
    rail_front = cx + P('ext_size') / 2                  # x = 300
    _, c = A.fresh_component(asm, 'PRT_SENSOR_NOSE')
    nose = A.rbox_y(lx - ld / 2 - 5, rail_front, plat_y0, plat_y1, -63, 63, 4.0)
    A.cut(nose, A.cyly(plat_y0 - 1, plat_y1 + 1, lx, 0, 14.0))                       # cable pass-through
    for sx in (-1, 1):
        for sz in (-1, 1):
            A.cut(nose, A.insert_hole((lx + sx * 28, plat_y1, sz * 28), (0, -1, 0), 'm25'))
    for s in (1, -1):
        hang = A.box(rail_front - 12, rail_front, plat_y0, rail_y0, s * 33, s * 63)
        A.union(hang, A.box(cx - 10, cx + 10, rail_y0 - 4, rail_y0, s * 33, s * 63))       # top flange under the rail
        A.cut(hang, A.hole((cx, rail_y0, s * 48), (0, 1, 0), P('bolt_m5_clr'), 4.0, over=1))
        for dz in (-P('cam_hx') / 2, P('cam_hx') / 2):
            for dy in (-P('cam_hy') / 2, P('cam_hy') / 2):
                A.cut(hang, A.insert_hole((rail_front, 172 + dy, s * 48 + dz), (-1, 0, 0), 'm25'))
        A.union(nose, hang)
    A.add_body(c, 'SENSOR_NOSE', nose, 'asa', 'ASA 4 walls; UV-stable')
    _, c = A.fresh_component(asm, 'BUY_LIDAR_FRONT')
    li = A.cyly(plat_y1, plat_y1 + lh, lx, 0, ld / 2)
    A.union(li, A.cyly(plat_y1 + lh, plat_y1 + lh + 4, lx, 0, ld / 2 - 12))
    A.add_body(c, 'LIDAR_FRONT', li, 'buy', 'YDLIDAR X2L stand-in [VERIFY]')
    _, c = A.fresh_component(asm, 'BUY_CAM_OV5647')
    for s, sn in ((1, 'STBD'), (-1, 'PORT')):
        A.add_body(c, 'CAM_' + sn, A.box(rail_front, rail_front + P('cam_H'), 172 - P('cam_W') / 2, 172 + P('cam_W') / 2,
                                          s * 48 - P('cam_L') / 2, s * 48 + P('cam_L') / 2), 'buy', 'OV5647 board')

    # ================= rear mast ==========================================================================
    mx = P('sen_mast_x'); top = P('sen_mast_top'); deck_y = P('ch_deckY')
    plat1 = top - lh; plat0 = plat1 - 8.0
    _, c = A.fresh_component(asm, 'PRT_LIDAR_MAST_REAR')
    base = A.rbox_y(-cx - 10, mx + 14, deck_y, deck_y + 4.0, -30, 30, 4.0)
    for sz in (-20, 20):
        A.cut(base, A.hole((-cx, deck_y + 4.0, sz), (0, -1, 0), P('bolt_m5_clr'), 4.0))
    mast = base
    A.union(mast, A.box(mx - 12, mx + 12, deck_y, plat1, -15, 15))
    for sz in (-1, 1):                                   # ribs
        A.union(mast, A.box(-cx - 8, mx + 12, deck_y, deck_y + 45, min(sz * 15, sz * 19), max(sz * 15, sz * 19)))
        A.union(mast, A.box(mx - 12, mx + 12, deck_y + 45, deck_y + 85, min(sz * 15, sz * 18), max(sz * 15, sz * 18)))
    A.union(mast, A.rbox_y(mx - ld / 2, mx + ld / 2, plat0, plat1, -ld / 2, ld / 2, 4.0))
    A.cut(mast, A.cyly(plat0 - 1, plat1 + 1, mx, 0, 10.0))
    for sx in (-1, 1):
        for sz in (-1, 1):
            A.cut(mast, A.insert_hole((mx + sx * 28, plat1, sz * 28), (0, -1, 0), 'm25'))
    A.add_body(c, 'LIDAR_MAST_REAR', mast, 'asa', 'ASA 4 walls; PA-CF optional')
    _, c = A.fresh_component(asm, 'BUY_LIDAR_REAR')
    li = A.cyly(plat1, top, mx, 0, ld / 2)
    A.union(li, A.cyly(top, top + 4, mx, 0, ld / 2 - 12))
    A.add_body(c, 'LIDAR_REAR', li, 'buy', 'YDLIDAR X2L stand-in [VERIFY]')

    # ================= REF_SWEEP + intrusion check ==============================================================
    sw = A.fresh_component(A.root(), 'REF_SWEEP')
    sx0, sx1, hw, sy0, sy1 = P('sweep_x0'), P('sweep_x1'), P('sweep_halfW'), deck_y + 10.0, P('sweep_top')
    A.add_body(sw[1], 'REF_SWEEP_VOLUME', A.box(sx0, sx1, sy0, sy1, -hw, hw), 'red', 'reference only (arm keep-out)')
    sw[0].isLightBulbOn = False
    bad = []
    for o in A.root().allOccurrences:
        if o.component.name.startswith(('REF_', 'ASM_ARM', 'ASM_DECK')):
            continue
        for b in o.component.bRepBodies:
            bb = b.boundingBox
            lo = o.transform.copy(); 
            pts = [adsk.core.Point3D.create(x, y, z) for x in (bb.minPoint.x, bb.maxPoint.x) for y in (bb.minPoint.y, bb.maxPoint.y) for z in (bb.minPoint.z, bb.maxPoint.z)]
            for p in pts:
                p.transformBy(o.transform)
            xs = [p.x * 10 for p in pts]; ys = [-p.y * 10 for p in pts]; zs = [p.z * 10 for p in pts]   # ys = sim z, zs = sim y
            if min(xs) < sx1 and max(xs) > sx0 and min(ys) < hw and max(ys) > -hw and min(zs) < sy1 and max(zs) > sy0:
                bad.append(o.component.name + '/' + b.name)
    print('keep-out intrusions:', bad if bad else 'none')
