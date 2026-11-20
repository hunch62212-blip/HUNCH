# 95_pose.py - drives both arms and the ACM through the 5 poses (stow, pregrasp, pick, apex, place) and ends at stow.
# Poses are root-proxy transform overrides (lib apply_pose), joint angles come from the closed-form 2-link IK
# (lib arm_pose_q).  Change POSES / SEQ below, or use  `python3 tools/fusion_mcp.py pose <name>`  for a single pose + `shot`.
import aegis_cad as A

SEQ = ['pick', 'pregrasp', 'apex', 'place', 'stow']

def run(_context: str):
    if not A.doc_ok():
        return
    for p in SEQ:
        q = A.apply_pose(p)
        print('pose %-9s q1,q2,q3 (deg) = %s   ACM base centre = %s' % (p, q, A.acm_pose_xy(p)))
