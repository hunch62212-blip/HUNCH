# 01_params.py - upsert every row of cad/fusion/params.csv into the Fusion User Parameters.
# (The MCP client embeds params.csv as AEGIS_FILES['params.csv'] because Fusion cannot read iCloud Drive.)
import csv, io
import adsk.core, adsk.fusion
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    des = A.design()
    ups = des.userParameters
    rows = list(csv.DictReader(io.StringIO(AEGIS_FILES['params.csv'])))
    n_ok = n_err = n_new = 0
    for r in rows:
        name, unit, expr, cm = r['Name'], r['Unit'], r['Expression'], r['Comments']
        p = ups.itemByName(name)
        try:
            if p is None:
                ups.add(name, adsk.core.ValueInput.createByString(expr), unit, cm)
                n_new += 1
            else:
                p.expression = expr
                p.comment = cm
            n_ok += 1
        except Exception as e:
            n_err += 1
            print('ERR', name, expr, '->', str(e)[:140])
    print('%d params (%d new), %d errors' % (n_ok, n_new, n_err))
    # spot-check derived values
    for k in ('arm_plate_gap', 'mot_z0', 'mot_z_face', 'rb_face_z', 'jb_t', 'wh_outer_z', 'dk_bush_d'):
        print(' ', k, '=', A.P(k))
