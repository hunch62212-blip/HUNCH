# 10_chassis.py - ASM_CHASSIS: 2020 frame (2 x 600 rails, 3 x 560 cross rails with centre bore + T-slots),
# 4 mm deck sheet with M5 bolt pattern, 8 printed corner brackets, tub (floor + 2 walls, sheet stock) with the
# rocker-shaft hole, joint-block bolt holes and tub-angle holes, 6 printed tub angles (M3 heat-set inserts).
# Deck top = ch_deckY.  All dimensions from User Parameters; re-run to rebuild.
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_CHASSIS')
    hl, hw = P('ch_halfLen'), P('ch_halfWid')
    ext = P('ext_size')
    y1 = P('ch_rail_y1'); y0 = y1 - ext
    rz = P('ch_rail_z'); cxe = P('ch_cross_x')
    Lr, Lc = P('ext_rail_L') / 2, P('ext_cross_L') / 2
    deck_t = P('ch_deck_t'); deck_y = P('ch_deckY')
    tz, tl, tt, ty0 = P('ch_tubZ'), P('ch_tubL') / 2, P('ch_tub_t'), P('ch_tubY0')
    h = ext / 2

    # ---- extrusions ------------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'BUY_EXT_2020')
    for nm, s in (('RAIL_STBD', 1), ('RAIL_PORT', -1)):
        A.add_body(c, nm, A.ext_profile('x', -Lr, Lr, y0, y1, s * rz - h, s * rz + h), 'alu', '2020 V-slot 600 mm')
    for nm, x in (('CROSS_FRONT', cxe), ('CROSS_MID', 0.0), ('CROSS_REAR', -cxe)):
        A.add_body(c, nm, A.ext_profile('z', x - h, x + h, y0, y1, -Lc, Lc), 'alu', '2020 V-slot 560 mm')

    # ---- deck ------------------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'BUY_DECK')
    deck = A.box(-hl, hl, deck_y - deck_t, deck_y, -hw, hw)
    pitch = P('ch_deck_bolt_pitch')
    xs = [-250 + pitch * i for i in range(int(500 / pitch) + 1)]
    n_m5 = 0
    for s in (1, -1):                       # along the long rails
        for x in xs:
            A.cut(deck, A.hole((x, deck_y, s * rz), (0, -1, 0), P('bolt_m5_clr'), deck_t)); n_m5 += 1
    for x in (cxe, 0.0, -cxe):              # along the cross rails
        for z in (-200, -100, 0, 100, 200):
            A.cut(deck, A.hole((x, deck_y, z), (0, -1, 0), P('bolt_m5_clr'), deck_t)); n_m5 += 1
    for z in (-20, 20):                     # rear LiDAR mast base (45_sensors) into the rear cross rail
        A.cut(deck, A.hole((-cxe, deck_y, z), (0, -1, 0), P('bolt_m5_clr'), deck_t)); n_m5 += 1
    n_m3 = 0
    for bx in A.tub_angle_x():              # tub angle M3 screws (countersink not modelled)
        for s in (1, -1):
            for dx in (-8, 8):
                A.cut(deck, A.hole((bx + dx, deck_y, s * (tz - 15)), (0, -1, 0), P('bolt_m3_clr'), deck_t)); n_m3 += 1
    for (x, z) in A.deck_mount_pts():
        A.cut(deck, A.hole((x, deck_y, z), (0, -1, 0), P('bolt_m3_clr'), deck_t)); n_m3 += 1
    A.add_body(c, 'BUY_DECK_SHEET', deck, 'wood', '4 mm plywood / 3 mm ACM deck; %d M5 + %d M3 holes' % (n_m5, n_m3))

    # ---- corner brackets (8) ------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'PRT_CORNER_BRKT')
    bt, arm, bw = P('ch_brkt_t'), P('ch_brkt_arm'), P('ch_brkt_w')
    by0, by1 = y0 - bt, y0
    def bracket(name, cx, cz, sx, dz):
        b = A.box(cx, cx + sx * arm, by0, by1, cz - bw / 2, cz + bw / 2)                # arm under the long rail
        A.union(b, A.box(cx - bw / 2, cx + bw / 2, by0, by1, cz, cz + dz * arm))      # arm under the cross rail
        for k in (12.0, 32.0):
            A.cut(b, A.hole((cx + sx * k, by0, cz), (0, 1, 0), P('bolt_m5_clr'), bt))
            A.cut(b, A.hole((cx, by0, cz + dz * k), (0, 1, 0), P('bolt_m5_clr'), bt))
        A.add_body(c, name, b, 'petg', 'PETG, 4 walls, flat')
    for sxe in (1, -1):
        for s in (1, -1):
            bracket('CORNER_%s%s' % ('F' if sxe > 0 else 'R', 'S' if s > 0 else 'P'), sxe * cxe, s * rz, -sxe, -s)
    for s in (1, -1):
        for sx in (1, -1):
            bracket('MIDJOINT_%s%s' % ('F' if sx > 0 else 'R', 'S' if s > 0 else 'P'), 0.0, s * rz, sx, -s)

    # ---- tub ---------------------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'BUY_TUB')
    floor = A.box(-tl, tl, ty0 - tt, ty0, -(tz + tt), tz + tt)
    for (x, z) in A.floor_bolts():
        A.cut(floor, A.hole((x, ty0, z), (0, -1, 0), P('bolt_m3_clr'), tt))
    A.add_body(c, 'TUB_FLOOR', floor, 'wood', '4 mm sheet')
    px, py = P('rb_P_x'), P('rb_P_y')
    for nm, s in (('TUB_WALL_STBD', 1), ('TUB_WALL_PORT', -1)):
        w = A.box(-tl, tl, ty0, y1 + deck_t - deck_t, s * tz, s * (tz + tt))
        A.cut(w, A.hole((px, py, s * tz), (0, 0, s), 10.0, tt))                           # rocker shaft clearance
        for dx in (-14, 14):
            for dy in (-14, 14):
                A.cut(w, A.hole((px + dx, py + dy, s * tz), (0, 0, s), P('bolt_m3_clr'), tt))   # joint block bolts
        for bx in A.tub_angle_x():
            for yy in (192.0, 208.0):
                A.cut(w, A.hole((bx, yy, s * tz), (0, 0, s), P('bolt_m3_clr'), tt))
        A.add_body(c, nm, w, 'wood', '4 mm sheet')

    # ---- tub angles (6) ------------------------------------------------------------------------------------
    _, c = A.fresh_component(asm, 'PRT_TUB_ANGLE')
    at = P('ch_angle_t')
    for bx in A.tub_angle_x():
        for s, sn in ((1, 'S'), (-1, 'P')):
            b = A.box(bx - 15, bx + 15, 180, y1, s * (tz - at), s * tz)                   # plate against the wall
            A.union(b, A.box(bx - 15, bx + 15, y1 - at, y1, s * (tz - 30), s * tz))       # flange under the deck
            for yy in (192.0, 208.0):
                A.cut(b, A.insert_hole((bx, yy, s * tz), (0, 0, -s), 'm3'))
            for dx in (-8, 8):
                A.cut(b, A.insert_hole((bx + dx, y1, s * (tz - 15)), (0, -1, 0), 'm3'))
            A.add_body(c, 'TUB_ANGLE_%s_%d' % (sn, int(bx)), b, 'petg', 'PETG, 4 walls, flat')
    for cc in asm.occurrences:
        print(cc.component.name, [(b.name) for b in cc.component.bRepBodies][:12])
