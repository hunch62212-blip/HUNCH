"""AEGIS-V1 prototype CAD helper library (Fusion 360 Python API).

Frames
------
Scripts work in the SIM frame, in millimetres:  +X forward, +Y up, +Z right, y = 0 is the ground.
Fusion is Z-up, so every point goes through V(): sim (x, y, z) mm -> Fusion (x, -z, y) cm.
(Proper rotation: det = +1, so handedness is preserved.)  Fusion internal unit is cm; never pass
raw mm to the API.

Geometry is built as temporary B-Rep solids (box / cyl / union / cut) and committed with add_body(),
which wraps them in a base feature.  Every dimension comes from User Parameters via P(); re-running a
script rebuilds its components from the current parameter values.

Import in a script (the MCP sends code text, so __file__ is unavailable):
    CAD_DIR = r'<repo>/cad/fusion'
    sys.path.insert(0, CAD_DIR + '/lib'); import aegis_cad as A; importlib.reload(A)
"""
import adsk.core, adsk.fusion, math, json, os

app = adsk.core.Application.get()
TBM = adsk.fusion.TemporaryBRepManager.get()
MM = 0.1                       # mm -> cm
DOC_PREFIX = 'AEGIS_PROTO'
DOC_NAME = 'AEGIS_PROTO_v1_8'

# --------------------------------------------------------------------------- document / params
def design():
    return adsk.fusion.Design.cast(app.activeProduct)

def root():
    return design().rootComponent

def doc_ok():
    """True when the active document is the AEGIS design (prints an error otherwise)."""
    d = app.activeDocument
    if d is None or not d.name.startswith(DOC_PREFIX) or design() is None:
        print('ERROR: active document is not %s*; run 00_new_design.py first' % DOC_PREFIX)
        return False
    return True

def P(name):
    """User parameter as a python float: mm for length params, degrees for angle params, raw otherwise."""
    p = design().userParameters.itemByName(name)
    if p is None:
        raise KeyError('missing user parameter ' + name)
    u = p.unit
    if u == 'mm':
        return p.value * 10.0
    if u == 'cm':
        return p.value * 10.0
    if u == 'm':
        return p.value * 1000.0
    if u in ('deg', '°'):
        return math.degrees(p.value)
    return p.value

def pexpr(name):
    return name   # parameter name usable directly in ValueInput.createByString

# --------------------------------------------------------------------------- coordinates
def V(x, y, z):
    """sim mm -> Fusion Point3D (cm)."""
    return adsk.core.Point3D.create(x * MM, -z * MM, y * MM)

def VV(x, y, z):
    """sim direction -> Fusion Vector3D (unit not enforced)."""
    return adsk.core.Vector3D.create(x, -z, y)

AX = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}

def mat(tx=0.0, ty=0.0, tz=0.0, rx=0.0, ry=0.0, rz=0.0, rots=None):
    """Matrix3D for an occurrence: rotations (deg, about SIM axes through the origin) applied in order, then a
    world translation (mm).  Use rots=[('z', a), ('y', 180)] for an explicit order (first item applied first);
    otherwise rx, ry, rz are applied in that order."""
    seq = rots if rots is not None else [('x', rx), ('y', ry), ('z', rz)]
    o = adsk.core.Point3D.create(0, 0, 0)
    m = adsk.core.Matrix3D.create()
    for ax, ang in seq:
        if ang:
            r = adsk.core.Matrix3D.create()
            r.setToRotation(math.radians(ang), VV(*AX[ax]), o)
            m.transformBy(r)          # m = r * m  -> later rotations act in the world frame (applied after earlier ones)
    m.translation = adsk.core.Vector3D.create(tx * MM, -tz * MM, ty * MM)
    return m

# --------------------------------------------------------------------------- appearances
COLORS = {
    'orange': (255, 110, 0), 'grey': (140, 140, 145), 'darkgrey': (70, 70, 75), 'yellow': (255, 200, 0),
    'white': (240, 240, 240), 'gold': (212, 175, 55), 'red': (210, 25, 25), 'black': (25, 25, 28),
    'petg': (60, 130, 175), 'asa': (45, 45, 50), 'tpu': (230, 230, 60), 'green': (30, 150, 70),
    'alu': (190, 195, 200), 'steel': (120, 125, 135), 'wood': (200, 160, 100), 'rubber': (35, 35, 38),
    'blue': (40, 90, 220), 'pcb': (20, 110, 60),
}
_BASE = {'blue_t': 'Plastic - Translucent Glossy (Blue)'}

def appearance(key):
    """Design-local appearance (created on demand). key: COLORS name, or 'buy' for translucent blue."""
    d = design()
    nm = 'AEG_' + key
    a = d.appearances.itemByName(nm)
    if a is not None:
        return a
    lib = app.materialLibraries.itemByName('Fusion Appearance Library')
    if key == 'buy':
        base = lib.appearances.itemByName(_BASE['blue_t'])
        return d.appearances.addByCopy(base, nm)
    base = lib.appearances.itemByName('Plastic - Matte (White)')
    a = d.appearances.addByCopy(base, nm)
    r, g, b = COLORS[key]
    for prop in a.appearanceProperties:
        if prop.name == 'Color':
            cp = adsk.core.ColorProperty.cast(prop)
            if cp:
                cp.value = adsk.core.Color.create(r, g, b, 255)
                break
    return a

# --------------------------------------------------------------------------- solids (temporary B-Rep)
def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box in sim mm (any order of bounds)."""
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    c = V((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    o = adsk.core.OrientedBoundingBox3D.create(c, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
                                               (x1 - x0) * MM, (z1 - z0) * MM, (y1 - y0) * MM)
    return TBM.createBox(o)

def cyl(p0, p1, r):
    """Cylinder from sim point p0 to p1 (mm), radius r."""
    return TBM.createCylinderOrCone(V(*p0), r * MM, V(*p1), r * MM)

def cone(p0, r0, p1, r1):
    return TBM.createCylinderOrCone(V(*p0), r0 * MM, V(*p1), r1 * MM)

def cylx(x0, x1, y, z, r): return cyl((x0, y, z), (x1, y, z), r)
def cyly(y0, y1, x, z, r): return cyl((x, y0, z), (x, y1, z), r)
def cylz(z0, z1, x, y, r): return cyl((x, y, z0), (x, y, z1), r)

def sphere(c, r):
    return TBM.createSphere(V(*c), r * MM)

def copy(b):
    return TBM.copy(b)

def union(a, *bs):
    for b in bs:
        if b is not None:
            TBM.booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a

def cut(a, *bs):
    for b in bs:
        if b is not None:
            TBM.booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a

def inter(a, b):
    TBM.booleanOperation(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return a

def move(b, tx=0.0, ty=0.0, tz=0.0, rx=0.0, ry=0.0, rz=0.0):
    """Transform a temp body in place (rotate about the sim origin first, then translate). Returns the body."""
    TBM.transform(b, mat(tx, ty, tz, rx, ry, rz))
    return b

def rot_about(b, axis, deg, center):
    """Rotate temp body about a sim axis line through center (mm)."""
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(deg), VV(*AX[axis]), V(*center))
    TBM.transform(b, m)
    return b

def rbox_z(x0, x1, y0, y1, z0, z1, r):
    """Box with its four edges parallel to Z rounded (radius r) - plate outlines."""
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    r = min(r, (x1 - x0) / 2.0, (y1 - y0) / 2.0)
    if r <= 0.01:
        return box(x0, x1, y0, y1, z0, z1)
    b = box(x0 + r, x1 - r, y0, y1, z0, z1)
    union(b, box(x0, x1, y0 + r, y1 - r, z0, z1))
    for cx in (x0 + r, x1 - r):
        for cy_ in (y0 + r, y1 - r):
            union(b, cylz(z0, z1, cx, cy_, r))
    return b

def rbox_x(x0, x1, y0, y1, z0, z1, r):
    """Box with edges parallel to X rounded."""
    r = min(r, (y1 - y0) / 2.0, (z1 - z0) / 2.0)
    b = box(x0, x1, y0 + r, y1 - r, z0, z1)
    union(b, box(x0, x1, y0, y1, z0 + r, z1 - r))
    for cy_ in (y0 + r, y1 - r):
        for cz in (z0 + r, z1 - r):
            union(b, cylx(x0, x1, cy_, cz, r))
    return b

def rbox_y(x0, x1, y0, y1, z0, z1, r):
    """Box with edges parallel to Y rounded (plan-view rounded rectangle)."""
    x0, x1 = sorted((x0, x1)); z0, z1 = sorted((z0, z1))
    r = min(r, (x1 - x0) / 2.0, (z1 - z0) / 2.0)
    b = box(x0 + r, x1 - r, y0, y1, z0, z1)
    union(b, box(x0, x1, y0, y1, z0 + r, z1 - r))
    for cx in (x0 + r, x1 - r):
        for cz in (z0 + r, z1 - r):
            union(b, cyly(y0, y1, cx, cz, r))
    return b

def bar_xy(p0, p1, w, z0, z1):
    """Stadium-shaped bar in the sim XY plane: ends at p0/p1 (mm), width w, extruded z0..z1."""
    (ax, ay), (bx, by) = p0, p1
    L = math.hypot(bx - ax, by - ay)
    ang = math.degrees(math.atan2(by - ay, bx - ax))
    b = box(0, L, -w / 2.0, w / 2.0, z0, z1)
    union(b, cylz(z0, z1, 0, 0, w / 2.0), cylz(z0, z1, L, 0, w / 2.0))
    rot_about(b, 'z', ang, (0, 0, 0))
    return move(b, ax, ay, 0)

def tube_x(x0, x1, y, z, ro, ri):
    b = cylx(x0, x1, y, z, ro)
    return cut(b, cylx(x0 - 1, x1 + 1, y, z, ri))

def tube_z(z0, z1, x, y, ro, ri):
    b = cylz(z0, z1, x, y, ro)
    return cut(b, cylz(z0 - 1, z1 + 1, x, y, ri))

def tube_y(y0, y1, x, z, ro, ri):
    b = cyly(y0, y1, x, z, ro)
    return cut(b, cyly(y0 - 1, y1 + 1, x, z, ri))

# --------------------------------------------------------------------------- holes, inserts, bearings
INS = {'m3': 0, 'm25': 0, 'm4': 0, 'm5': 0, 'brg': 0}      # counters, flushed to body attributes by add_body

def hole(p, d, dia, depth, over=0.6):
    """Cutting cylinder starting at surface point p, going along unit sim direction d for `depth` mm
    (and `over` mm back out of the surface)."""
    p0 = (p[0] - d[0] * over, p[1] - d[1] * over, p[2] - d[2] * over)
    p1 = (p[0] + d[0] * depth, p[1] + d[1] * depth, p[2] + d[2] * depth)
    return cyl(p0, p1, dia / 2.0)

def cbore(p, d, dia, depth, cdia, cdepth):
    """Clearance hole with counterbore (head recess) - dia through `depth`, cdia for the first cdepth."""
    h = hole(p, d, dia, depth)
    return union(h, hole(p, d, cdia, cdepth))

def insert_hole(p, d, size='m3', extra=0.0):
    """Blind heat-set insert hole: depth = insert length + 1 mm (plan 1.3), no chamfer."""
    INS[size] += 1
    return hole(p, d, P('ins_%s_d' % size), P('ins_%s_depth' % size) + extra)

def insert_boss(p, d, size='m3', length=None):
    """Solid boss around an insert hole: OD ins_*_boss (>= insert OD + 2*wall_min), starting at p going along d."""
    od = P('ins_%s_boss' % size) if size != 'm25' else P('ins_m25_d') + 2 * P('wall_min')
    L = length if length is not None else P('ins_%s_depth' % size) + 1.0
    p1 = (p[0] + d[0] * L, p[1] + d[1] * L, p[2] + d[2] * L)
    return cyl(p, p1, od / 2.0)

def bearing_seat_cut(body, p, d, depth=None, ribs=6, thru_dia=None, thru_depth=None):
    """608 press seat cut into `body` at surface point p going along unit direction d.
    Bore = brg_od + 2*clr_press + 2*crush_rib; `ribs` axial crush ribs (inner edge at the nominal press
    radius, i.e. interference = crush_rib at the ribs) are added back.  A shaft through-hole (thru_dia,
    default brg_id + 2) continues behind the seat for thru_depth (default 14 mm).  Returns body."""
    depth = depth if depth is not None else P('brg_w')
    rr = (P('brg_od') + 2 * P('clr_press')) / 2.0
    rib_r = 0.8
    ax = [i for i in range(3) if abs(d[i]) > 0.5][0]
    oth = [i for i in range(3) if i != ax]
    sgn = 1 if d[ax] > 0 else -1
    cut(body, hole(p, d, 2 * (rr + P('crush_rib')), depth))
    for k in range(ribs):
        a = 2 * math.pi * k / ribs
        c = [p[0], p[1], p[2]]
        c[oth[0]] += (rr + rib_r) * math.cos(a)
        c[oth[1]] += (rr + rib_r) * math.sin(a)
        q1 = list(c); q1[ax] += sgn * depth
        union(body, cyl(tuple(c), tuple(q1), rib_r))
    th = [p[0], p[1], p[2]]
    th[ax] += sgn * (depth - 0.01)
    cut(body, hole(tuple(th), d, thru_dia or (P('brg_id') + 2.0), thru_depth or 14.0, over=0))
    INS['brg'] += 1
    return body

# --------------------------------------------------------------------------- components
import re as _re
def bn(name):
    """Component base name: Fusion appends ' (1)' when a deleted component of that name is still in the undo history."""
    return _re.sub(r' \(\d+\)$', '', name)

def find_occ(parent_comp, name):
    for o in parent_comp.occurrences:
        if bn(o.component.name) == name:
            return o
    return None

def asm(name):
    """Top-level ASM_ component occurrence (find or create), identity transform. Returns component."""
    o = find_occ(root(), name)
    if o is None:
        o = root().occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.component.name = name
    return o.component

def fresh_component(parent_comp, name, m=None):
    """Delete every occurrence of a component called `name` under parent_comp and create it anew
    (identity transform).  Returns (occurrence, component).  Idempotent re-run helper."""
    for o in list(parent_comp.occurrences):
        if bn(o.component.name) == name:
            o.deleteMe()
    o = parent_comp.occurrences.addNewComponent(m if m is not None else adsk.core.Matrix3D.create())
    o.component.name = name
    return o, o.component


def place(parent_comp, comp, m):
    """Add another occurrence of an existing component with matrix m (from mat())."""
    return parent_comp.occurrences.addExistingComponent(comp, m)

def add_body(comp, name, tb, appear=None, material_note=None):
    """Commit a temporary body to `comp` as a named solid (base feature).  Flushes insert counters to
    body attributes group 'aegis'.  Returns the BRepBody."""
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    b = comp.bRepBodies.add(tb, bf)
    b.name = name
    bf.finishEdit()
    b.name = name
    for bb in comp.bRepBodies:         # re-find (the body object can be stale after finishEdit)
        if bb.name == name:
            b = bb
    if appear:
        b.appearance = appearance(appear) if isinstance(appear, str) else appear
    for k, v in INS.items():
        if v:
            b.attributes.add('aegis', 'n_' + k, str(v))
        INS[k] = 0
    if material_note:
        b.attributes.add('aegis', 'material', material_note)
    return b

def bbox_mm(body):
    """Sim-frame bounding box (x0,x1,y0,y1,z0,z1) in mm of a BRepBody (world space of its component)."""
    bb = body.boundingBox
    lo, hi = bb.minPoint, bb.maxPoint
    zs = sorted((-hi.y * 10, -lo.y * 10))
    return (lo.x * 10, hi.x * 10, lo.z * 10, hi.z * 10, zs[0], zs[1])

def dims_sorted(body):
    b = bbox_mm(body)
    return sorted((b[1] - b[0], b[3] - b[2], b[5] - b[4]), reverse=True)

# --------------------------------------------------------------------------- bed split
def bed_fits(dims, bed=None, bedz=None):
    """Plan 7.4 rule: sorted dims a>=b>=c: (b,c <= bed and a <= bed_z) or (a,b <= bed)."""
    bed = bed if bed is not None else P('bed_max')
    bedz = bedz if bedz is not None else P('bed_z')
    a, b, c = dims
    return (b <= bed and c <= bed and a <= bedz) or (a <= bed and b <= bed)

def split_keyed(tb, axis, at, keys, key_w=14.0, key_h=6.0, key_depth=10.0, bolt='m3', toward=1, bolt_len=30.0):
    """Split a temp body at the plane axis=at into (half_lo, half_hi) joined by tongue-and-groove keys.
    `keys` = list of (u, v) positions in the plane (the two non-split axes in x,y,z order).  The tongue
    (key_w x key_h, protruding key_depth into half_hi) is part of half_lo; the groove in half_hi is
    clr_snug oversized per side.  Each key also gets an M3 bolt along the split axis: clearance hole
    through half_lo's tongue, heat-set insert hole in half_hi.  Returns (lo, hi).
    NOTE: in the plane, key_w runs along the first in-plane axis, key_h along the second."""
    ai = 'xyz'.index(axis)
    oth = [i for i in range(3) if i != ai]
    big = 5000.0
    def half(sign):
        lo = [-big, -big, -big]; hi = [big, big, big]
        if sign < 0:
            hi[ai] = at
        else:
            lo[ai] = at
        return box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    lo_b = inter(copy(tb), half(-1))
    hi_b = inter(copy(tb), half(+1))
    snug = P('clr_snug')
    for (u, v) in keys:
        def mk(w, h, d0, d1, grow=0.0):
            mn = [0, 0, 0]; mx = [0, 0, 0]
            mn[ai] = at + d0; mx[ai] = at + d1
            mn[oth[0]] = u - w / 2 - grow; mx[oth[0]] = u + w / 2 + grow
            mn[oth[1]] = v - h / 2 - grow; mx[oth[1]] = v + h / 2 + grow
            return box(mn[0], mx[0], mn[1], mx[1], mn[2], mx[2])
        union(lo_b, mk(key_w, key_h, -0.5, key_depth))
        cut(hi_b, mk(key_w, key_h, -0.5, key_depth + snug, grow=snug))
        # M3 bolt along the split axis: clearance hole through the tongue and into half_lo, heat-set insert in half_hi
        d = [0, 0, 0]; d[ai] = 1
        tip = [0, 0, 0]; tip[ai] = at + key_depth; tip[oth[0]] = u; tip[oth[1]] = v
        cut(lo_b, hole(tuple(tip), tuple(-x for x in d), P('bolt_m3_clr'), key_depth + bolt_len, over=0))
        cut(hi_b, insert_hole(tuple(tip), tuple(d), 'm3'))
    return lo_b, hi_b

# --------------------------------------------------------------------------- misc
def report(comp, indent=''):
    for b in comp.bRepBodies:
        print(indent + b.name, 'vol cm3=%.2f' % b.volume, 'dims', ['%.1f' % x for x in dims_sorted(b)])

def set_view(eye, target, up=(0, 0, 1), fit=True, ortho=False):
    """Camera in FUSION (cm) coordinates - convenience for screenshots."""
    vp = app.activeViewport
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType if ortho else adsk.core.CameraTypes.PerspectiveCameraType
    cam.eye = adsk.core.Point3D.create(*eye)
    cam.target = adsk.core.Point3D.create(*target)
    cam.upVector = adsk.core.Vector3D.create(*up)
    vp.camera = cam
    if fit:
        vp.fit()
    app.activeViewport.refresh()

# --------------------------------------------------------------------------- shared layout geometry
def wheels():
    """The six wheels: dict name -> {x, y, z, side(+1 = right/+z), kind, pivot, arm_ang, link}.
    arm_ang = direction (deg, world XY plane) from the wheel centre toward its link pivot (rocker P for F*, bogie B for M*/R*).
    FL/FR/RL/RR are driven; ML/MR are passive idlers."""
    wb, r, tz = P('wb_x'), P('wh_r'), P('tr_z')
    Pp = (P('rb_P_x'), P('rb_P_y')); Bp = (P('rb_B_x'), P('rb_B_y'))
    out = {}
    for tag, x, piv, kind, link in (('F', wb, Pp, 'drive', 'rocker'), ('M', 0.0, Bp, 'idler', 'bogie'), ('R', -wb, Bp, 'drive', 'bogie')):
        for sn, side in (('L', -1), ('R', 1)):
            out[tag + sn] = dict(x=x, y=r, z=side * tz, side=side, kind=kind, pivot=piv, link=link,
                                 arm_ang=math.degrees(math.atan2(piv[1] - r, piv[0] - x)))
    return out

def pod_mat(w):
    """Occurrence matrix for a pod-frame component (origin = wheel centre, +z outboard, local +x = arm toward the
    link pivot).  Right side: rotate about z by arm_ang.  Left side: rotate about z by 180-arm_ang, then flip about y."""
    if w['side'] > 0:
        rots = [('z', w['arm_ang'])]
    else:
        rots = [('z', 180.0 - w['arm_ang']), ('y', 180.0)]
    return mat(w['x'], w['y'], w['z'], rots=rots)

def floor_bolts():
    """(x, z) of the M3 bolts through the tub floor: battery tray (4) and electronics tray halves (4 each)."""
    out = []
    bx, bl = P('bat_x'), P('bat_L')
    for sx in (-1, 1):
        for sz in (-1, 1):
            out.append((bx + sx * (bl / 2 + 8), sz * 58.5))
    ex = P('ele_x')
    for sx in (-1, 1):
        for zz in (6.0, 137.0, -6.0, -137.0):
            out.append((ex + sx * 45.0, zz))
    return out

def tub_angle_x():
    return (-150.0, 80.0, 160.0)

def ext_profile(body_axis, x0, x1, y0, y1, z0, z1):
    """2020 extrusion with centre bore and 4 T-slots, along `body_axis` ('x' or 'z'). Bounds in sim mm."""
    b = box(x0, x1, y0, y1, z0, z1)
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    s = max(x1 - x0, z1 - z0) if True else 0
    if body_axis == 'x':
        L0, L1 = x0, x1
        cut(b, cylx(L0 - 1, L1 + 1, cy, cz, 2.1))
        for sy, sz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ey, ez = cy + sy * 10, cz + sz * 10
            # slot opening 6.2 wide x 2 deep + chamber 11 wide x 4.5 deep
            def slot(w, d0, d1):
                if sy:
                    return box(L0 - 1, L1 + 1, ey - sy * d1, ey - sy * d0, cz - w / 2, cz + w / 2)
                return box(L0 - 1, L1 + 1, cy - w / 2, cy + w / 2, ez - sz * d1, ez - sz * d0)
            cut(b, slot(6.2, -1, 2.0), slot(11.0, 2.0, 6.5))
    else:
        L0, L1 = z0, z1
        cut(b, cylz(L0 - 1, L1 + 1, cx, cy, 2.1))
        for sy, sx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ey, ex = cy + sy * 10, cx + sx * 10
            def slot(w, d0, d1):
                if sy:
                    return box(cx - w / 2, cx + w / 2, ey - sy * d1, ey - sy * d0, L0 - 1, L1 + 1)
                return box(ex - sx * d1, ex - sx * d0, cy - w / 2, cy + w / 2, L0 - 1, L1 + 1)
            cut(b, slot(6.2, -1, 2.0), slot(11.0, 2.0, 6.5))
    return b

def place_pods(asm, occ0, comp, base_name, keys, wmap=None):
    """Instance a pod-frame component at the wheels in `keys` (e.g. ['FL','FR']).  occ0 must have been created with
    fresh_component(asm, name, pod_mat(wmap[keys[0]])) - nested occurrences cannot be moved afterwards."""
    wmap = wmap or wheels()
    for i, k in enumerate(keys):
        o = occ0 if i == 0 else place(asm, comp, pod_mat(wmap[k]))
        try:
            o.name = '%s_%s' % (base_name, k)
        except Exception:
            pass

def rect_xy(p, ang_deg, s0, s1, w, z0, z1):
    """Rectangle (no rounded ends) in the sim XY plane starting at point p, running along direction ang_deg from
    distance s0 to s1, width w, extruded z0..z1 (mm)."""
    b = box(s0, s1, -w / 2.0, w / 2.0, z0, z1)
    rot_about(b, 'z', ang_deg, (0, 0, 0))
    return move(b, p[0], p[1], 0)

def dir_xy(ang_deg):
    return (math.cos(math.radians(ang_deg)), math.sin(math.radians(ang_deg)))

# --------------------------------------------------------------------------- arm geometry (P2b / P3c)
# Link frames: origin on the joint axis, local +x along the link, local y = in-plane perpendicular, z = lateral
# (bodies are modelled directly at sim z = side * d, d = |lateral|).  A link is placed with link_mat().
def arm_layers():
    """Lateral layer stack (d = |z|, mm) of one arm.  Shaft axis of the big servos = their H dimension."""
    t, cw, c, H = P('arm_plate_t'), P('arm_cw'), P('clr_slip'), P('srv_big_H')
    ht = P('arm_horn_t')
    stack = t + 0.3 + cw + c + H + c + 0.3 + t
    a = P('arm_z') - stack / 2.0
    L = {}
    L['uin'] = (a, a + t)
    L['wall'] = (a + t + 0.3, a + t + 0.3 + cw)
    L['srv'] = (L['wall'][1] + c, L['wall'][1] + c + H)
    L['uout'] = (L['srv'][1] + c + 0.3, L['srv'][1] + c + 0.3 + t)
    L['horn'] = (L['uout'][1] + 0.4, L['uout'][1] + 0.4 + ht)
    L['fout'] = (L['horn'][1] + 0.2, L['horn'][1] + 0.2 + t)
    L['fin'] = (a - 0.8 - t, a - 0.8)
    Hs = P('srv_std_H')
    L['wsrv'] = (L['fin'][1] + 0.2, L['fin'][1] + 0.2 + Hs)           # wrist servo (shaft face against F_in)
    L['whorn'] = (L['fin'][0] - 0.4 - ht, L['fin'][0] - 0.4)
    L['hook'] = (L['whorn'][0] - P('hook_t') - 0.2, L['whorn'][0] - 0.2)
    L['gate'] = (L['hook'][0] - 0.3 - P('gate_t'), L['hook'][0] - 0.3)
    L['hsrv'] = (L['hook'][1] + 0.2, L['hook'][1] + 0.2 + P('srv_mic_H'))   # latch servo outboard of the hook plate
    return L

SERVO = {'big': 'srv_big_', 'std': 'srv_std_', 'mic': 'srv_mic_'}
def servo_spec(kind):
    p = SERVO[kind]
    g = lambda n, dflt=None: P(p + n) if n in ('L', 'W', 'H', 'holeX') or kind != 'mic' else dflt
    sp = dict(L=P(p + 'L'), W=P(p + 'W'), H=P(p + 'H'), hx=P(p + 'holeX'))
    if kind == 'mic':
        sp.update(fl=sp['L'] + 10.0, fz=P(p + 'H') - 10.0, hy=0.0, off=6.0)
    else:
        sp.update(fl=P(p + 'flangeL'), fz=P(p + 'flange_z'), hy=P(p + 'holeY'), off=P(p + 'shaft_off'))
    return sp

def servo_model(kind, shx, shy, d_top, dirn, s, bdir=-1):
    """Stylised servo (temp body).  Shaft axis along lateral z at (shx, shy); shaft face at |z| = d_top, body extends
    toward `dirn` (+1 increasing |z|, -1 decreasing).  Body length runs from the shaft toward bdir (+-1) along local x."""
    sp = servo_spec(kind)
    x0, x1 = (shx - (sp['L'] - sp['off']), shx + sp['off']) if bdir < 0 else (shx - sp['off'], shx + (sp['L'] - sp['off']))
    dd = lambda u: d_top + dirn * u
    body = box(x0, x1, shy - sp['W'] / 2, shy + sp['W'] / 2, s * dd(0), s * dd(sp['H']))
    ear_u = sp['H'] - sp['fz']
    xc = (x0 + x1) / 2
    union(body, box(xc - sp['fl'] / 2, xc + sp['fl'] / 2, shy - sp['W'] / 2, shy + sp['W'] / 2, s * dd(ear_u), s * dd(ear_u + 2.5)))
    union(body, cylz(s * dd(0), s * dd(-4.0), shx, shy, 3.0))                 # spline stub
    return body, (xc, ear_u + 2.5, sp)

def arm_geom():
    S = (P('arm_sh_x'), P('arm_sh_y'))
    return S, P('arm_L1'), P('arm_L2')

def ik2(tx, ty, up=True):
    """Two-link IK for the wrist axis (tx, ty).  Returns (q1, q2) rad; picks the elbow-up (higher elbow) solution."""
    S, L1, L2 = arm_geom()
    dx, dy = tx - S[0], ty - S[1]
    d = math.hypot(dx, dy)
    D = max(-1.0, min(1.0, (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    best = None
    for sg in (-1, 1):
        q2 = sg * math.acos(D)
        q1 = math.atan2(dy, dx) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        ey = S[1] + L1 * math.sin(q1)
        if best is None or (ey > best[0]) == up:
            best = (ey, q1, q2)
    return best[1], best[2]

def arm_pose_q(pose):
    """Joint angles (q1, q2, q3) rad for a named pose; the hook seat (trunnion) hits the pose target with the mouth up (psi = 0)."""
    ho = P('hook_off')
    T = {'stow': (P('stow_x'), P('stow_y')), 'pregrasp': (P('pick_x'), 90.0 - 50.0), 'pick': (P('pick_x'), 90.0),
         'apex': (P('arm_sh_x'), 515.0), 'place': (P('dk_place_x'), P('ch_deckY') + P('acm_trun_y'))}
    tx, ty = T[pose]
    q1, q2 = ik2(tx - ho, ty)
    q3 = math.degrees(-q1 - q2)
    q3 = ((q3 - 60.0) % 360.0) + 60.0           # unwrap into the 60..420 deg window (one servo sweep covers all poses)
    return q1, q2, math.radians(q3)

def acm_pose_xy(pose):
    """ACM base-centre (x, y) for a pose (ACM carried by the hook from apex on; on the ground / deck otherwise)."""
    if pose in ('pick', 'pregrasp'):
        return P('pick_x'), 0.0
    if pose == 'place':
        return P('dk_place_x'), P('ch_deckY')
    if pose == 'apex':
        return P('arm_sh_x'), 515.0 - P('acm_trun_y')
    return P('pick_x'), 0.0

def link_mats(q):
    """Occurrence matrices of links 1..3 for joint angles q (rad)."""
    S, L1, L2 = arm_geom()
    a1 = q[0]; a2 = q[0] + q[1]; a3 = q[0] + q[1] + q[2]
    E = (S[0] + L1 * math.cos(a1), S[1] + L1 * math.sin(a1))
    W = (E[0] + L2 * math.cos(a2), E[1] + L2 * math.sin(a2))
    deg = math.degrees
    return [mat(S[0], S[1], 0, rots=[('z', deg(a1))]), mat(E[0], E[1], 0, rots=[('z', deg(a2))]),
            mat(W[0], W[1], 0, rots=[('z', deg(a3))])], E, W

def deck_mount_pts():
    """(x, z) of the M3 bolts through the deck sheet for tower bases, stow cradles, locating pins and the deck latch."""
    out = []
    ar = arm_layers()
    sx = P('arm_sh_x')
    for s in (-1, 1):
        for x in (sx - 58.0, sx + 23.0):
            for d in (ar['wall'][0] + 3.0, ar['srv'][1] - 3.0):
                out.append((x, s * d))
        for x in (P('stow_x') - 12.0, P('stow_x') + 12.0):
            for d in (134.0, 148.0):
                out.append((x, s * d))
    pc = P('dk_place_x')
    for sg in (-1, 1):
        for k in range(3):
            a = math.radians(90 + 120 * k)
            out.append((pc + sg * P('dk_pin_span') / 2 + 10 * math.cos(a), 10 * math.sin(a)))
    xl = P('dk_latch_x')
    for dx in (-17.0, 17.0):
        for dz in (-30.0, 30.0):
            out.append((xl + dx, dz))
    return out

def servo_cradle(kind, shx, shy, d_top, dirn, s, bdir, d_far, ins='m3'):
    """Block (temp) that holds a servo by its ears: solid from d_far to the ledge on the bulk side of the ears,
    body pocket (clr_slip per side) through the ledge, and heat-set insert holes in the ledge for the ear screws.
    Returns (solid, pocket, ledge_d, [(x, y), ...] insert positions)."""
    sp = servo_spec(kind); c = P('clr_slip')
    x0, x1 = (shx - (sp['L'] - sp['off']), shx + sp['off']) if bdir < 0 else (shx - sp['off'], shx + (sp['L'] - sp['off']))
    xc = (x0 + x1) / 2.0
    ear_u = sp['H'] - sp['fz']
    ledge = d_top + dirn * (ear_u + 2.5 + c)
    solid = box(xc - sp['fl'] / 2 - 3, xc + sp['fl'] / 2 + 3, shy - sp['W'] / 2 - 4, shy + sp['W'] / 2 + 4, s * d_far, s * ledge)
    pocket = box(x0 - c, x1 + c, shy - sp['W'] / 2 - c, shy + sp['W'] / 2 + c, s * (d_top + dirn * (sp['H'] + c)), s * (ledge - dirn * 2.0))
    into = 1 if d_far > ledge else -1
    ys = [shy - sp['hy'] / 2, shy + sp['hy'] / 2] if sp['hy'] > 0 else [shy]
    pts = [(xc + sx_ * sp['hx'] / 2, y) for sx_ in (-1, 1) for y in ys]
    size = 'm25' if kind == 'mic' else 'm3'
    for (x, y) in pts:
        cut(solid, insert_hole((x, y, s * ledge), (0, 0, s * into), size))
    return solid, pocket, ledge, pts

def hook_geom():
    """Hook-frame geometry (origin = wrist axis, +x along the forearm, mouth opens toward +y)."""
    ho = P('hook_off'); rt = P('trun_d') / 2.0
    half = math.radians(P('hook_seat_ang') / 2.0)
    yA = -rt / math.sin(half)                       # V apex
    throat = P('hook_mouth') / 2.0
    y_throat = yA + throat / math.tan(half)         # where the flanks reach the mouth half-width
    return dict(ho=ho, rt=rt, yA=yA, throat=throat, y_throat=y_throat, tan_c=math.tan(math.radians(P('hook_cheek_ang'))),
                pivot=(ho + 34.0, rt + P('latch_gap') + P('gate_t') / 2.0))

def poly_prism_z(pts, z0, z1):
    """Prism of a polygon given in sim XY (mm) extruded z0..z1: union of triangles fan from pts[0] (convex pieces via boxes is
    avoided: Fusion temp bodies are built from a swept profile through the sketch-free path - here we fan convex quads)."""
    # build by wedge cuts: intersection of half-planes applied to a big box -> only valid for CONVEX polygons
    big = 2000.0
    z0, z1 = min(z0, z1), max(z0, z1)
    b = box(-big, big, -big, big, z0, z1)
    n = len(pts)
    for i in range(n):
        (ax, ay), (bx, by) = pts[i], pts[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        # keep the left side of the directed edge (CCW polygon): remove the right side with a large box rotated onto the edge
        L = math.hypot(dx, dy)
        ang = math.degrees(math.atan2(dy, dx))
        cutter = box(-big, big, -big, 0, z0 - 1, z1 + 1)
        rot_about(cutter, 'z', ang, (0, 0, 0))
        move(cutter, ax, ay, 0)
        cut(b, cutter)
    return b

LINK_OF = {'PRT_UPPER_ARM': 0, 'BUY_ELBOW_SERVO': 0, 'BUY_SHOULDER_HORN': 0,
           'PRT_FORE_ARM': 1, 'BUY_WRIST_SERVO': 1, 'BUY_ELBOW_HORN': 1,
           'PRT_HOOK': 2, 'BUY_LATCH_SERVO': 2, 'BUY_WRIST_HORN': 2}
GATE = 'PRT_LATCH_GATE'

def _mclose(a, b, tol=1e-4):
    return all(abs(x - y) < tol for x, y in zip(a.asArray(), b.asArray()))

def _replace(parent, old, m):
    """Re-place a link occurrence at matrix m.  Fusion composes the matrix given to addExistingComponent with the transform of
    the existing occurrence, so the inverse of the old transform is folded in (tries both composition orders)."""
    comp = old.component
    inv = old.transform2.copy()
    inv.invert()
    for first in (True, False):
        a = m.copy(); b = inv.copy()
        if first:
            a.transformBy(b)
            cand = a
        else:
            b.transformBy(m)
            cand = b
        new = parent.occurrences.addExistingComponent(comp, cand)
        if _mclose(new.transform2, m):
            old.deleteMe()
            return new
        new.deleteMe()
    raise RuntimeError('could not place %s at the requested matrix' % comp.name)

def apply_pose(name, gate_open=None):
    """Move both arms (links re-placed by delete+add) and the ACM to a named pose.  Returns the joint angles (deg)."""
    q = arm_pose_q(name)
    mats, E, W = link_mats(q)
    if gate_open is None:
        gate_open = (name == 'pregrasp')
    g = hook_geom()
    # NOTE: as-built joints (90_assembly_joints.py) use fixed sketch-point origins, so a chained joint pose is not valid;
    # poses are applied by re-placing the link occurrences, which requires removing the joints (re-run 90 afterwards).
    for jn in list(all_joints()):
        jn.deleteMe()
    # transform overrides can only be set on root-context proxies (root.allOccurrences)
    for o in list(root().allOccurrences):
        fp = o.fullPathName
        if not (fp.startswith('ASM_ARM_R:') or fp.startswith('ASM_ARM_L:')) or o.assemblyContext is None:
            continue
        base = bn(o.component.name).rsplit('_', 1)[0]
        if base in LINK_OF:
            o.transform2 = mats[LINK_OF[base]]
        elif base == GATE:
            m = adsk.core.Matrix3D.create()
            if gate_open:
                m.setToRotation(math.radians(-90.0), VV(0, 0, 1), V(g['pivot'][0], g['pivot'][1], 0))
            m.transformBy(mats[2])
            o.transform2 = m
    ax, ay = acm_pose_xy(name)
    for o in root().occurrences:
        if o.component.name == 'ASM_ACM':
            o.transform2 = mat(ax, ay, 0)
    app.activeViewport.refresh()
    return [round(math.degrees(v), 1) for v in q]

# --------------------------------------------------------------------------- joints (P3c)
def all_joints():
    return [j for j in root().allAsBuiltJoints] if hasattr(root(), 'allAsBuiltJoints') else list(root().asBuiltJoints)

def get_joint(name):
    for j in all_joints():
        if j.name == name:
            return j
    return None

def find_prox(comp_name, asm_prefix=None):
    """All assembly-context occurrence proxies (root.allOccurrences) of a component name (prefix match, e.g. 'WHEEL_DRIVEN')."""
    out = []
    for o in root().allOccurrences:
        if (o.component.name == comp_name or o.component.name.startswith(comp_name + ' (')) and (asm_prefix is None or o.fullPathName.startswith(asm_prefix)):
            out.append(o)
    return out

def occ_origin_sim(o):
    """World position (sim mm) of an occurrence proxy's component origin."""
    t = o.transform2.translation
    return (t.x * 10.0, t.z * 10.0, -t.y * 10.0)       # fusion (x, y, z) cm -> sim (x, y=fz, z=-fy)

def joint_sketch():
    for sk in list(root().sketches):
        if sk.name == 'J_POINTS':
            sk.deleteMe()
    sk = root().sketches.add(root().xZConstructionPlane)
    sk.name = 'J_POINTS'
    return sk

def joint_pt(sk, x, y):
    """Sketch point at sim (x, y) (the revolute axis is the sim z direction = Fusion -Y)."""
    p = sk.modelToSketchSpace(adsk.core.Point3D.create(x * MM, 0.0, y * MM))
    return sk.sketchPoints.add(adsk.core.Point3D.create(p.x, p.y, 0))

def joint_sketch_y():
    for sk in list(root().sketches):
        if sk.name == 'J_POINTS_Y':
            sk.deleteMe()
    sk = root().sketches.add(root().xYConstructionPlane)
    sk.name = 'J_POINTS_Y'
    return sk

def joint_pt_y(sk, x, z):
    """Sketch point at sim (x, z) for a vertical (sim y) revolute axis (Fusion Z)."""
    return sk.sketchPoints.add(adsk.core.Point3D.create(x * MM, -z * MM, 0))

def make_joint(name, moving, fixed, pt=None, lo=None, hi=None, vertical=False):
    """As-built joint: revolute about sim z through sketch point pt (limits in deg, sim sign), or rigid when pt is None."""
    g = adsk.fusion.JointGeometry.createByPoint(pt) if pt is not None else None
    inp = root().asBuiltJoints.createInput(moving, fixed, g)
    if pt is not None:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection if vertical else adsk.fusion.JointDirections.YAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root().asBuiltJoints.add(inp)
    j.name = name
    if pt is not None and lo is not None:
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True; lim.isMaximumValueEnabled = True
        if vertical:
            lim.minimumValue = math.radians(lo); lim.maximumValue = math.radians(hi)
        else:
            lim.minimumValue = math.radians(-hi); lim.maximumValue = math.radians(-lo)       # Fusion +Y = sim -z
    return j
