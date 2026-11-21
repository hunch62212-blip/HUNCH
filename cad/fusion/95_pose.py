# 95_pose.py - drives both arms and the ACM through the 5 poses (stow, pregrasp, pick, apex, place) and ends at stow.
# Poses are root-proxy transform overrides (lib apply_pose), joint angles come from the closed-form 2-link IK
# (lib arm_pose_q).  Change POSES / SEQ below, or use  `python3 tools/fusion_mcp.py pose <name>`  for a single pose + `shot`.
import aegis_cad as A

# Sim reference (src/sim/config.js ARM, arm.js fk): stow q = [2.8, -2.8, 1.2], seat (161.5, 342.0) mm, psi 1.2 rad;
# place q = [1.880, 2.113, 2.290] (q2 > 0 branch) at DECK.placeX -42.6 mm.  CAD q differ slightly: the seat is hook_off ahead of the wrist.

SEQ = ['pick', 'pregrasp', 'apex', 'place', 'stow']

def run(_context: str):
    if not A.doc_ok():
        return
    for p in SEQ:
        q = A.apply_pose(p)
        print('pose %-9s q1,q2,q3 (deg) = %s   ACM base centre = %s' % (p, q, A.acm_pose_xy(p)))
