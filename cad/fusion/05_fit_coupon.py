# 05_fit_coupon.py - PRT_FIT_COUPON: print this FIRST (PLA ok for the fit test, PETG for the final numbers),
# then write the chosen clearances back into params.csv (clr_slip / clr_snug / clr_press / ins_m3_d).
#   plate : row 1  seven holes  d = peg_d + 2*c     c = 0, .05, .10, .15, .20, .25, .30   (slip / snug / press)
#           row 2  seven M3 heat-set insert holes   d = 3.8 .. 4.4 (0.1 steps), depth = ins_m3_depth
#           end    608 bearing press seat with crush ribs + shaft hole
#   pegs  : separate strip with seven d = 6 mm pegs (and two M3 / M4 dummy bosses) to try in the plate
# Lies on the ground beside the rover (z = +700 / +790) so it never overlaps the assembly.
import adsk.core, adsk.fusion
import aegis_cad as A

PEG_D = 6.0
CL = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
INS_D = [3.8, 3.9, 4.0, 4.1, 4.2, 4.3, 4.4]

def run(_context: str):
    if not A.doc_ok():
        return
    P = A.P
    occ, comp = A.fresh_component(A.root(), 'PRT_FIT_COUPON')
    z0 = 700.0
    t = max(P('ins_m3_depth') + 2.0, 9.0)            # plate thickness: insert depth + 2 mm floor
    L, W = 210.0, 56.0
    plate = A.rbox_y(-L / 2, L / 2, 0, t, z0 - W / 2, z0 + W / 2, 4.0)
    xs = [-84 + 24 * k for k in range(7)]
    for x, c in zip(xs, CL):
        A.cut(plate, A.hole((x, t, z0 + 14), (0, -1, 0), PEG_D + 2 * c, t + 1))
    for x, d in zip(xs, INS_D):
        A.cut(plate, A.hole((x, t, z0 - 14), (0, -1, 0), d, P('ins_m3_depth')))
    A.bearing_seat_cut(plate, (88, t, z0), (0, -1, 0))
    A.add_body(comp, 'PRT_FIT_COUPON_PLATE', plate, 'petg', 'PLA/PETG 0.16 mm layers')

    z1 = z0 + 90.0
    strip = A.rbox_y(-L / 2 + 6, L / 2 - 6, 0, 3, z1 - 9, z1 + 9, 3.0)
    for x in xs:
        A.union(strip, A.cyly(3, 3 + 8, x, z1, PEG_D / 2))
    # dummy bosses to check the M3 / M4 insert boss sizes
    A.union(strip, A.cyly(3, 3 + 8, 88, z1, P('ins_m3_boss') / 2))
    A.union(strip, A.cyly(3, 3 + 8, 70, z1, P('ins_m4_boss') / 2))
    A.add_body(comp, 'PRT_FIT_COUPON_PEGS', strip, 'petg', 'PLA/PETG 0.16 mm layers')
    A.report(comp)
    print('coupon at z = %.0f..%.0f mm' % (z0 - W / 2, z1 + 9))
    occ.isLightBulbOn = False      # parked off to the side; hidden so viewport fits frame the rover (toggle on to inspect)
