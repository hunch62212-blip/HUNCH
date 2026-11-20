#!/usr/bin/env python3
"""Tiny client for the local Autodesk Fusion MCP server (http://127.0.0.1:27182/mcp).

Usage:
  fusion_mcp.py list                                  # list tools
  fusion_mcp.py read '{"queryType":"document","operation":"open"}'
  fusion_mcp.py script path/to/script.py [--readonly]  # runs def run(_context) in Fusion
  fusion_mcp.py cam ex ey ez tx ty tz                  # camera in the SIM frame (mm), then use `shot out.png`
  fusion_mcp.py shot out.png [direction]               # screenshot (iso-top-right etc.)
  fusion_mcp.py pose stow|pregrasp|pick|apex|place     # move arms + ACM
  fusion_mcp.py vis hide|show|only|showall names...
  fusion_mcp.py stlcopy                                 # /tmp/aegis_stl -> cad/fusion/out/stl (after 98_export.py)
  fusion_mcp.py call <toolName> '<json args>'
"""
import base64, glob, json, os, sys, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CAD_ROOT = os.path.dirname(HERE)          # .../cad/fusion


def prelude():
    """Fusion's Python is sandboxed (no access to iCloud Drive / ~/Library/Mobile Documents), so scripts cannot
    import lib/ or read params.csv themselves.  This client embeds them in the script text it sends:
      * every lib/*.py becomes an importable module in sys.modules (import aegis_cad as A just works);
      * AEGIS_FILES = {'params.csv': <text>, ...} holds the *.csv files of cad/fusion/ (global in the script)."""
    parts = ["import sys, types"]
    for f in sorted(glob.glob(os.path.join(CAD_ROOT, "lib", "*.py"))):
        name = os.path.splitext(os.path.basename(f))[0]
        src = open(f).read()
        parts.append("_m = types.ModuleType(%r); exec(compile(%r, %r, 'exec'), _m.__dict__); sys.modules[%r] = _m"
                     % (name, src, f, name))
    files = {os.path.basename(f): open(f).read() for f in glob.glob(os.path.join(CAD_ROOT, "*.csv"))}
    parts.append("AEGIS_FILES = %r" % files)
    return "\n".join(parts) + "\n"

URL = "http://127.0.0.1:27182/mcp"
HDR = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}


def _post(body, sid=None, timeout=600):
    h = dict(HDR)
    if sid:
        h["MCP-Session-Id"] = sid
    req = urllib.request.Request(URL, data=json.dumps(body).encode(), headers=h, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode("utf-8", "replace")
        sid = r.headers.get("MCP-Session-Id", sid)
    if raw.strip().startswith("event:") or raw.strip().startswith("data:"):
        data = [l[5:].strip() for l in raw.splitlines() if l.startswith("data:")]
        raw = data[-1] if data else "{}"
    return (json.loads(raw) if raw.strip() else {}), sid


def session():
    _, sid = _post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "hunch22", "version": "1"}}})
    _post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    return sid


def call(tool, args):
    sid = session()
    res, _ = _post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": tool, "arguments": args}}, sid)
    return res


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__); return
    cmd = a[0]
    if cmd == "list":
        sid = session()
        res, _ = _post({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}, sid)
        print(json.dumps(res, indent=1)); return
    if cmd == "read":
        res = call("fusion_mcp_read", json.loads(a[1]))
    elif cmd == "script":
        code = prelude() + open(a[1]).read()
        obj = {"script": code}
        if "--readonly" in a:
            obj["readOnly"] = True
        res = call("fusion_mcp_execute", {"featureType": "script", "object": obj})
    elif cmd == "cam":
        # cam ex ey ez tx ty tz   (SIM frame, mm: +x fwd, +y up, +z right) - sets the viewport camera (no fit)
        e = [float(v) for v in a[1:4]]; t = [float(v) for v in a[4:7]]
        f = lambda p: (p[0] * 0.1, -p[2] * 0.1, p[1] * 0.1)
        code = ("import adsk.core\ndef run(_c):\n    app = adsk.core.Application.get(); vp = app.activeViewport; c = vp.camera\n"
                "    c.isSmoothTransition = False\n    c.cameraType = adsk.core.CameraTypes.PerspectiveCameraType\n"
                "    c.target = adsk.core.Point3D.create(*%r)\n    c.eye = adsk.core.Point3D.create(*%r)\n"
                "    c.upVector = adsk.core.Vector3D.create(0, 0, 1)\n    vp.camera = c\n    vp.fit()\n    vp.refresh()\n    print('cam ok')\n" % (f(t), f(e)))
        res = call("fusion_mcp_execute", {"featureType": "script", "object": {"script": code}})
    elif cmd == "pose":
        # pose stow|pregrasp|pick|apex|place  - moves both arms and the ACM (lib: apply_pose)
        code = prelude() + "import aegis_cad as A\ndef run(_c):\n    print('pose', %r, A.apply_pose(%r))\n" % (a[1], a[1])
        res = call("fusion_mcp_execute", {"featureType": "script", "object": {"script": code}})
    elif cmd == "stlcopy":
        # copy the STLs written by 98_export.py (/tmp/aegis_stl) into cad/fusion/out/stl (Fusion cannot write into iCloud)
        import shutil
        dst = os.path.join(CAD_ROOT, "out", "stl"); os.makedirs(dst, exist_ok=True)
        for f in os.listdir(dst):
            os.remove(os.path.join(dst, f))
        n = 0
        for f in sorted(os.listdir("/tmp/aegis_stl")):
            shutil.copy(os.path.join("/tmp/aegis_stl", f), dst); n += 1
        print("copied", n, "files to", dst); return
    elif cmd == "vis":
        # vis hide|show|only name [name...]  - toggles visibility of components (by component name) / bodies (by body name);
        # `vis showall` shows everything except PRT_FIT_COUPON and REF_* (use `vis show REF_SWEEP`).  Names are substrings ('BUY_DECK' hides BUY_DECK).
        mode, names = a[1], a[2:]
        code = ("import adsk.core, adsk.fusion\nNAMES = %r\nMODE = %r\ndef run(_c):\n"
                "    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)\n"
                "    hit = lambda n: any(k in n for k in NAMES)\n"
                "    for o in d.rootComponent.allOccurrences:\n"
                "        if o.component.name.startswith('ASM_') or o.component.name == 'PRT_FIT_COUPON': continue\n"
                "        if MODE == 'showall': o.isLightBulbOn = not o.component.name.startswith('REF_')\n"
                "        elif MODE == 'only': o.isLightBulbOn = hit(o.component.name) or any(hit(b.name) for b in o.component.bRepBodies)\n"
                "        elif hit(o.component.name): o.isLightBulbOn = (MODE == 'show')\n"
                "        for b in o.component.bRepBodies:\n"
                "            if MODE != 'only' and hit(b.name): b.isLightBulbOn = (MODE == 'show')\n"
                "            if MODE == 'showall': b.isLightBulbOn = True\n"
                "    for o in d.rootComponent.allOccurrences:\n"
                "        if o.component.name == 'PRT_FIT_COUPON': o.isLightBulbOn = False\n"
                "    print('vis', MODE, NAMES)\n" % (names, mode))
        res = call("fusion_mcp_execute", {"featureType": "script", "object": {"script": code}})
    elif cmd == "shot":
        args = {"queryType": "screenshot", "width": 1600, "height": 1000, "transparentBackground": False}
        if len(a) > 2:
            args["direction"] = a[2]
        res = call("fusion_mcp_read", args)
        for c in res.get("result", {}).get("content", []):
            if c.get("type") == "image":
                open(a[1], "wb").write(base64.b64decode(c["data"])); print("saved", a[1]); return
            if c.get("type") == "text":
                try:
                    j = json.loads(c["text"])
                    if "base64Data" in j:
                        open(a[1], "wb").write(base64.b64decode(j["base64Data"])); print("saved", a[1]); return
                except Exception:
                    pass
        print(json.dumps(res)[:2000]); return
    elif cmd == "call":
        res = call(a[1], json.loads(a[2]))
    else:
        print(__doc__); return
    out = res.get("result", res)
    for c in out.get("content", []) if isinstance(out, dict) else []:
        if c.get("type") == "text":
            print(c["text"])
        else:
            print("[", c.get("type"), "content ]")
    if not (isinstance(out, dict) and out.get("content")):
        print(json.dumps(res, indent=1)[:5000])


if __name__ == "__main__":
    main()
