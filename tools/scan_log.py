"""Record and explain the scans printed by src/main.py (DEBUG = True).

    python3 tools/scan_log.py                  # listen on the Brain's USB port, Ctrl-C to stop
    python3 tools/scan_log.py logs/run.log     # explain a saved log (or text pasted from the VEX terminal)

Close the VEX terminal in VS Code before listening: only one program can hold the port.
Needs: pip install pyserial matplotlib

For every scan it prints the spacing between new distance readings, each surface it saw and why the detector accepted or rejected
it, and it runs the real CubeDetector from src/main.py on the same data.
"""
import glob
import os
import statistics
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "sim"))
from run import load_program  # noqa: E402  (src/main.py with the desktop VEX API)


def record():
    import serial
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* port: is the Brain connected by USB?")
    os.makedirs("logs", exist_ok=True)
    name = os.path.join("logs", time.strftime("%Y%m%d-%H%M%S") + ".log")
    out = open(name, "w")
    lock = threading.Lock()

    def listen(port):       # the Brain has two ports; the user port carries print()
        try:
            with serial.Serial(port, 115200, timeout=1) as s:
                while True:
                    line = s.readline().decode("utf-8", "replace").strip()
                    if line and all(31 < ord(c) < 127 for c in line):
                        with lock:
                            out.write(line + "\n")
                            out.flush()
                            if line.split(",")[0] not in AREAS:
                                print(line)
        except Exception as e:  # noqa: BLE001 - one port failing must not stop the other
            print("%s: %s" % (port, e))

    for port in ports:
        threading.Thread(target=listen, args=(port,), daemon=True).start()
    print("saving to %s - run the program, Ctrl-C when done" % name)
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    out.close()
    return name


AREAS = ("MINE", "DISPOSE", "STORE", "OUT")


def read_scans(path):
    """[(samples [(line number, sensor field X, raw)], None), ...]: MINE lines between SEARCHING and PICK/DONE"""
    scans, cur, scanning = [], [], False
    for n, line in enumerate(open(path, errors="replace")):
        line = line.strip()
        parts = line.split(",")
        if line.startswith("SEARCHING"):
            scanning, cur = True, []
            scans.append((cur, None))
        elif line.startswith(("PICK", "DONE", "ERROR")):
            scanning = False
        if scanning and parts[0] == "MINE" and len(parts) == 5:
            try:
                x, raw = float(parts[2]), 0.0 if parts[4] == "-" else float(parts[4])
            except ValueError:
                continue
            cur.append((n, x, raw))
        elif line and parts[0] not in AREAS:
            print("  brain: " + line)
    return [scan for scan in scans if scan[0]]


def surfaces(g, samples):
    """Split readings into surfaces the way CubeDetector does, with the verdict."""
    near_limit = g["WALL_Z"] - g["CUBE_SIZE"] / 2
    pts = [(x, raw - g["GRAB_DISTANCE"] if 0 < raw <= 1000 else None) for _, x, raw in samples]
    out, i = [], 0
    while i < len(pts):
        if pts[i][1] is None or pts[i][1] >= near_limit:
            i += 1
            continue
        first = i
        i += 1
        while i < len(pts) and pts[i][1] is not None and abs(pts[i][1] - pts[i - 1][1]) <= g["DEPTH_JUMP"]:
            i += 1
        zs = [p[1] for p in pts[first:i]]
        right = (pts[first - 1][0] + pts[first][0]) / 2 if first else pts[first][0]
        left = (pts[i - 1][0] + pts[i][0]) / 2 if i < len(pts) else pts[i - 1][0]
        width = right - left
        need = min(zs) + g["GRIP_DEPTH"] + 5
        why = []
        if not g["CUBE_WIDTH_MIN"] <= width <= g["CUBE_WIDTH_MAX"]:
            why.append("width %.0f not in %.0f-%.0f" % (width, g["CUBE_WIDTH_MIN"], g["CUBE_WIDTH_MAX"]))
        for side, j in (("right", first - 1), ("left", i)):
            if not 0 <= j < len(pts):
                why.append("%s edge outside the scan" % side)
            elif pts[j][1] is None:
                why.append("%s gap reads nothing (None): detector treats that as blocked" % side)
            elif pts[j][1] < need:
                why.append("%s gap only %.0f deep, needs %.0f" % (side, pts[j][1], need))
        out.append((left, right, min(zs), max(zs), why))
    return out


def replay(g, samples):
    """Feed the real CubeDetector at 1 mm steps; each reading is held until the next."""
    det, found = g["CubeDetector"](), []
    end = [(0, samples[-1][1] - 3.0, 0)]     # unchanged readings are not printed: hold the last one 3 mm
    for (_, x0, raw), (_, x1, _) in zip(samples, samples[1:] + end):
        z = raw - g["GRAB_DISTANCE"] if 0 < raw <= 1000 else None
        x = x0
        while x > x1:
            hit = det.add(x, z)     # already the field X of the beam
            if hit and not any(abs(hit[0] - f[0]) < g["CUBE_SIZE"] / 2 for f in found):
                found.append(hit)
            x -= 1.0
    return found


def explain(path):
    g = load_program()
    scans = read_scans(path)
    if not scans:
        sys.exit("no scan lines (area,arm X,sensor X,Z,distance) in %s" % path)
    for n, (s, summary) in enumerate(scans, 1):
        print("\n=== scan %d: %d new distance readings" % (n, len(s)))
        if summary:
            print("  " + summary)
        if len(s) < 3:
            continue
        step = statistics.median(a[1] - b[1] for a, b in zip(s, s[1:]))
        print("  %.1f mm between new readings (median), edge uncertainty about +-%.1f mm" % (step, step / 2))
        raws = [r for _, _, r in s]
        print("  raw readings: min %.0f, max %.0f mm; no object (0 or >1000): %d of %d"
              % (min(raws), max(raws), sum(1 for r in raws if not 0 < r <= 1000), len(raws)))
        for left, right, zmin, zmax, why in surfaces(g, s):
            print("  surface X %.0f..%.0f (width %.0f), grab-Z %.0f..%.0f: %s"
                  % (left, right, right - left, zmin, zmax, "; ".join(why) or "ACCEPTED"))
        found = replay(g, s)
        print("  real CubeDetector: " + (", ".join("cube at X %.0f Z %.0f" % f for f in found) or "nothing"))
        plot(g, s, n, path)


def plot(g, s, n, path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
    xs = [x for _, x, _ in s]
    ds = [r if 0 < r <= 1000 else float("nan") for _, _, r in s]
    plt.figure(figsize=(11, 4))
    plt.step(xs, ds, where="post", lw=1, label="distance reading")
    plt.axhline(g["WALL_Z"] - g["CUBE_SIZE"] / 2 + g["GRAB_DISTANCE"], color="r", ls="--", lw=1,
                label="farther than this = background (WALL_Z - CUBE_SIZE / 2)")
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.xlabel("sensor field X [mm] (scan runs left to right on this plot)")
    plt.ylabel("distance [mm]")
    plt.title("scan %d" % n)
    plt.legend(fontsize=8)
    out = "%s.scan%d.png" % (os.path.splitext(path)[0], n)
    plt.savefig(out, dpi=120, bbox_inches="tight")
    print("  plot: " + out)


if __name__ == "__main__":
    log = sys.argv[1] if len(sys.argv) > 1 else record()
    explain(log)
    try:
        import matplotlib.pyplot as plt
        plt.show()
    except ImportError:
        pass
