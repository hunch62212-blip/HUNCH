# 70_deck_guides.py - ASM_DECK_ACM: PRT_PIN_ROUND + PRT_PIN_DIAMOND (tapered locating pins at dk_place_x +- dk_pin_span/2, red,
# seat-switch pocket in each base), PRT_DECK_LATCH (MG996R rotary latch behind the ACM rear, arm closes over the keeper bar),
# PRT_STOW_CRADLE_R/L (post + pin the hook seats onto in stow, TPU sleeve).  REF_SWEEP is built by 45_sensors.py.
import math
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    asm = A.asm('ASM_DECK_ACM')
    for o in list(asm.occurrences):
        if o.component.name.startswith(('PRT_PIN', 'PRT_DECK_LATCH', 'PRT_DECK_LATCH_ARM', 'PRT_STOW', 'BUY_DECK_LATCH')):
            o.deleteMe()
    deck = P('ch_deckY'); pc = P('dk_place_x'); span = P('dk_pin_span')
    rb, rt, ph = P('dk_pin_base') / 2, P('dk_pin_tip') / 2, P('dk_pin_h')
    fl_t = 5.0
    # ---- locating pins ----------------------------------------------------------------------------------
    for nm, x, diamond in (('PRT_PIN_ROUND', pc + span / 2, False), ('PRT_PIN_DIAMOND', pc - span / 2, True)):
        _, comp = A.fresh_component(asm, nm)
        b = A.cyly(deck, deck + fl_t, x, 0, 15.0)
        pin = A.cone((x, deck + fl_t, 0), rb, (x, deck + fl_t + ph, 0), rt)
        if diamond:                                     # relieved flats along the pin line (x) -> only constrains rotation
            for sg in (-1, 1):
                A.cut(pin, A.box(x + sg * 3.2, x + sg * 20, deck + fl_t - 1, deck + fl_t + ph + 1, -20, 20))
        A.union(b, pin)
        for k in range(3):
            a = math.radians(90 + 120 * k)
            A.cut(b, A.hole((x + 10 * math.cos(a), deck + fl_t, 10 * math.sin(a)), (0, -1, 0), P('bolt_m3_clr'), fl_t))
        A.cut(b, A.box(x + 6, x + 16, deck + 1.0, deck + 4.2, -3.5, 3.5))                     # seat-switch pocket (side entry)
        A.add_body(comp, nm[4:], b, 'red', 'PETG; pins 100% infill')
    # ---- deck latch --------------------------------------------------------------------------------------
    xl = P('dk_latch_x')
    sp = A.servo_spec('std')
    _, comp = A.fresh_component(asm, 'PRT_DECK_LATCH')
    zb0, zb1 = -(sp['L'] - sp['off']), sp['off']
    zc = (zb0 + zb1) / 2
    base_t = 5.0; sv_y0 = deck + base_t + 3.0
    ear_y = sv_y0 + sp['fz']                          # ear plane (top of the ear) measured from the servo bottom
    led = ear_y - 2.5 - P('clr_slip')
    lb = A.rbox_y(xl - 19, xl + 19, deck, deck + base_t, zc - 42, zc + 42, 5.0)
    cr = A.box(xl - sp['W'] / 2 - 4, xl + sp['W'] / 2 + 4, deck + base_t, led, zc - sp['fl'] / 2 - 3, zc + sp['fl'] / 2 + 3)
    A.union(lb, cr)
    A.cut(lb, A.box(xl - sp['W'] / 2 - P('clr_slip'), xl + sp['W'] / 2 + P('clr_slip'), sv_y0 - P('clr_slip'), led + 2, zb0 - P('clr_slip'), zb1 + P('clr_slip')))
    for dx in (-sp['hy'] / 2, sp['hy'] / 2):
        for dz in (-sp['hx'] / 2, sp['hx'] / 2):
            A.cut(lb, A.insert_hole((xl + dx, led, zc + dz), (0, -1, 0), 'm3'))
    for (x, z) in A.deck_mount_pts():
        if abs(x - xl) < 20:
            A.cut(lb, A.hole((x, deck + base_t, z), (0, -1, 0), P('bolt_m3_clr'), base_t))
    A.add_body(comp, 'DECK_LATCH_BASE', lb, 'red', 'PETG 5 walls')
    top = sv_y0 + sp['H']
    arm = A.box(xl, xl + 36, top + 1.0, top + 6.0, -6, 6)
    A.union(arm, A.cyly(top + 1.0, top + 6.0, xl, 0, 9.0))
    A.union(arm, A.cyly(top + 6.0, top + 6.0 + 4.0, xl + 36 - 6, 0, 3.0))
    A.cut(arm, A.cyly(top, top + 7, xl, 0, 3.0))
    _, comp = A.fresh_component(asm, 'PRT_DECK_LATCH_ARM')
    A.add_body(comp, 'DECK_LATCH_ARM', arm, 'red', 'PETG; closes over the keeper bar')
    _, comp = A.fresh_component(asm, 'BUY_DECK_LATCH_SERVO')
    body = A.box(xl - sp['W'] / 2, xl + sp['W'] / 2, sv_y0, sv_y0 + sp['H'], zb0, zb1)
    A.union(body, A.box(xl - sp['W'] / 2, xl + sp['W'] / 2, ear_y - 2.5, ear_y, zc - sp['fl'] / 2, zc + sp['fl'] / 2))
    A.union(body, A.cyly(sv_y0 + sp['H'], sv_y0 + sp['H'] + 1.0, xl, 0, 3.0))
    A.add_body(comp, 'SERVO_DECK_LATCH', body, 'buy', 'MG996R [VERIFY]')
    # ---- stow cradles -----------------------------------------------------------------------------------
    # Wrist axis at stow = sim fk(stow q).tip / 8.16 (K) = (stow_x, stow_y) = (161.56, 342); the hook seat is hook_off further along psi = 1.2.  The post stands INBOARD of the hook / gate
    # plates (d 108..121, hook plate d 126..132, gate d 122..126) because the tower + wrist servo + forearm plates fill d >= 132 there;
    # a pin cantilevers outboard from the post through the V seat (TPU sleeve over d 124.5..133).
    sx, sy = A.stow_seat_xy()          # hook seat = wrist (161.56, 342) + hook_off along psi 1.2 = (173.9, 373.7)
    for s, tag in ((1, 'R'), (-1, 'L')):
        _, comp = A.fresh_component(asm, 'PRT_STOW_CRADLE_' + tag)
        d0, d1, dp = 108.0, 121.0, 133.0
        post = A.box(sx - 8, sx + 8, deck, sy, s * d0, s * d1)
        A.union(post, A.rbox_y(sx - 18, sx + 18, deck, deck + 5, s * 103.0, s * 125.0, 4.0))
        A.union(post, A.cylz(s * d1, s * dp, sx, sy, 4.5))                      # pin dia 9 under the 12 mm TPU sleeve
        for x in (sx - 12, sx + 12):
            for d in (109.0, 119.0):
                A.cut(post, A.hole((x, deck + 5, s * d), (0, -1, 0), P('bolt_m3_clr'), 5))
        A.cut(post, A.insert_hole((sx, sy, s * dp), (0, 0, -s), 'm3'))
        A.add_body(comp, 'STOW_POST_' + tag, post, 'yellow', 'PETG 100% infill')
        pad = A.tube_z(s * 124.5, s * dp, sx, sy, P('trun_d') / 2, 4.6)
        A.add_body(comp, 'STOW_PAD_' + tag, pad, 'tpu', 'TPU 95A sleeve')
        A.report(comp)
    for o in asm.occurrences:
        A.report(o.component)
