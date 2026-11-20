# 00_new_design.py - create (or reuse) the AEGIS_PROTO_v1_8 Fusion design document (Parametric + Hybrid, mm)
# and the empty ASM_ skeleton.  Idempotent.  Run:  python3 cad/fusion/tools/fusion_mcp.py script cad/fusion/00_new_design.py
import adsk.core, adsk.fusion
import aegis_cad as A

SKELETON = ['ASM_CHASSIS', 'ASM_SUSPENSION', 'ASM_DRIVE', 'ASM_WHEELS', 'ASM_POWER',
            'ASM_SENSORS', 'ASM_ARM_L', 'ASM_ARM_R', 'ASM_DECK_ACM']

def run(_context: str):
    app = adsk.core.Application.get()
    cur = app.activeDocument
    if cur is None or not cur.name.startswith(A.DOC_PREFIX):
        # close stray empty/unsaved scratch documents so only the AEGIS doc remains
        for d in list(app.documents):
            if d.name.startswith('Untitled') and not d.isSaved:
                d.close(False)
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        des = adsk.fusion.Design.cast(app.activeProduct)
        des.designType = adsk.fusion.DesignTypes.ParametricDesignType
        try:
            des.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
        except Exception as e:      # older builds: attribute missing
            print('designIntent not settable:', e)
        des.unitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits if hasattr(adsk.fusion, 'DistanceUnits') else des.unitsManager.distanceDisplayUnits
        saved = False
        for proj in app.data.dataProjects:
            try:
                doc.saveAs(A.DOC_NAME, proj.rootFolder, 'AEGIS-V1 arm-cargo rover prototype CAD (cad/fusion scripts)', 'P1d')
                print('saved into project:', proj.name)
                saved = True
                break
            except Exception as e:
                print('saveAs failed for project', proj.name, '->', str(e)[:120])
        if not saved:
            print('WARNING: document not saved (name stays', app.activeDocument.name, ')')
    des = adsk.fusion.Design.cast(app.activeProduct)
    print('doc:', app.activeDocument.name, '| designType', des.designType, '| length units', des.unitsManager.defaultLengthUnits)
    root = des.rootComponent
    for n in SKELETON:
        A.asm(n)
    print('components:', len(root.occurrences), [o.component.name for o in root.occurrences])
