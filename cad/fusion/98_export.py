# 98_export.py - one STL per PRT_ body (print orientation = modelled orientation; slice flat) into /tmp/aegis_stl,
# plus params_export.csv.  Fusion cannot write into iCloud Drive, so the repo copy is made by the client:
#     python3 tools/fusion_mcp.py script 98_export.py && python3 tools/fusion_mcp.py stlcopy
import os
import adsk.core, adsk.fusion
import aegis_cad as A

OUT = '/tmp/aegis_stl'

def run(_context: str):
    if not A.doc_ok():
        return
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    em = A.design().exportManager
    n = 0
    seen = set()
    for o in A.root().allOccurrences:
        c = o.component
        nm = A.bn(c.name)
        if not nm.startswith('PRT_') or nm in seen:
            continue
        seen.add(nm)
        for b in c.bRepBodies:
            fn = os.path.join(OUT, '%s__%s.stl' % (nm, b.name))
            opt = em.createSTLExportOptions(b, fn)
            opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            opt.isBinaryFormat = True
            em.execute(opt)
            n += 1
    with open(os.path.join(OUT, 'params_export.csv'), 'w') as f:
        f.write(AEGIS_FILES['params.csv'])
    print('exported %d STL files to %s' % (n, OUT))
