"""Record and explain the scans printed by src/main.py (DEBUG = True).

    python3 tools/scan_log.py                  # listen on the Brain's USB port; stops by itself at DONE/ERROR
    python3 tools/scan_log.py logs/run.log     # explain a saved log (or text pasted from the VEX terminal)

After the run it saves logs/<time>.log and plots the whole run (<log>.run.png) and every scan
(<log>.scanN.png), then opens the plots.

Quit VS Code (Cmd+Q) after downloading the program: its VEX extension keeps the Brain's print port
open even when the terminal panel is closed, and only one program can hold it.
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
    finished = threading.Event()
    busy = []

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
                            if line.startswith(("DONE", "ERROR", "STOP")):
                                finished.set()
        except Exception as e:  # noqa: BLE001 - one port failing must not stop the other
            if "busy" in str(e).lower():
                busy.append(port)
                finished.set()
            else:
                print("%s: %s" % (port, e))

    for port in ports:
        threading.Thread(target=listen, args=(port,), daemon=True).start()
    print("saving to %s - start the program on the Brain (stops at DONE/ERROR, or Ctrl-C)" % name)
    try:
        while not finished.is_set():
            time.sleep(0.2)
        time.sleep(1.0)             # catch the last lines
    except KeyboardInterrupt:
        pass
    with lock:
        out.close()
    if busy:
        os.remove(name)
        holder = os.popen("lsof %s 2>/dev/null | tail -n +2" % " ".join(
            p.replace("/cu.", "/tty.") + " " + p for p in busy)).read().strip()
        sys.exit("%s is used by another program:\n%s\nQuit VS Code (Cmd+Q) - its VEX extension holds the port "
                 "even with the terminal closed - then run this again." % (", ".join(busy), holder or "(unknown)"))
    return name


AREAS = ("MINE", "DISPOSE", "STORE", "OUT")


def read_scans(path):
    """[(samples [(line number, sensor field X, raw)], None), ...]: sensor lines between SEARCHING and PICK/DONE"""
    scans, cur, scanning = [], [], False
    for n, line in enumerate(open(path, errors="replace")):
        line = line.strip()
        parts = line.split(",")
        if line.startswith("SEARCHING"):
            scanning, cur = True, []
            scans.append((cur, None))
        elif line.startswith(("PICK", "DONE", "ERROR")):
            scanning = False
        if scanning and parts[0] in AREAS and len(parts) == 5:     # the scan starts in DISPOSE
            try:
                x, raw = float(parts[2]), 0.0 if parts[4] == "-" else float(parts[4])
            except ValueError:
                continue
            cur.append((n, x, raw))
        elif line and parts[0] not in AREAS:
            print("  brain: " + line)
    return [scan for scan in scans if scan[0]]


def surfaces(g, samples):
    """The detector's segments (same rules as CubeDetector), with the verdict for each near one."""
    floor = g["WALL_Z"] - 5
    segs, cur, last = [], None, None
    for _, x, raw in samples:
        if last is not None and last - x < g["SAMPLE_STEP"]:
            continue
        last = x
        z = raw - g["GRAB_DISTANCE"] if 0 < raw <= 1000 else 9999
        if cur and abs(z - sum(cur[2]) / len(cur[2])) <= g["JUMP_MM"]:
            cur[1], cur[2] = x, (cur[2] + [z])[-g["SMOOTH_COUNT"]:]
            cur[3].append(z)
            continue
        cur = [x, x, [z], [z]]
        segs.append(cur)
    out = []
    for n, (start, end, recent, zs) in enumerate(segs):
        avg = sum(zs) / len(zs)
        if avg >= floor:
            continue
        why = []
        if not g["CUBE_WIDTH_MIN"] <= start - end <= g["CUBE_WIDTH_MAX"]:
            why.append("length %.0f not in %.0f-%.0f" % (start - end, g["CUBE_WIDTH_MIN"], g["CUBE_WIDTH_MAX"]))
        for side, j in (("right", n - 1), ("left", n + 1)):
            if 0 <= j < len(segs) and sum(segs[j][3]) / len(segs[j][3]) <= avg:
                why.append("%s neighbour is nearer" % side)
        out.append((end, start, min(zs), max(zs), why))
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
    last = det.finish()                 # scan end: find_cube judges the last segment
    if last and not any(abs(last[0] - f[0]) < g["CUBE_SIZE"] / 2 for f in found):
        found.append(last)
    return found


def explain(path):
    g = load_program()
    plot_run(g, path)
    scans = read_scans(path)
    if not scans:
        print("no scan lines (MINE lines after SEARCHING) in %s" % path)
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
            print("  segment X %.0f..%.0f (length %.0f), grab-Z %.0f..%.0f: %s"
                  % (left, right, right - left, zmin, zmax, "; ".join(why) or "ACCEPTED"))
        start = g["SEARCH_START_X"] - g["SENSOR_X_OFFSET"] + 1.0
        found = replay(g, [r for r in s if r[1] <= start])      # the detector runs from the scan start on
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


def plot_run(g, path):
    """The whole run: distance vs sensor field X on top, the trolley path and events below."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
    rows, events, trip = [], [], 0
    for n, line in enumerate(open(path, errors="replace")):
        line = line.strip()
        parts = line.split(",")
        if parts[0] in AREAS and len(parts) == 5:
            try:
                arm, sensor, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            dist = float(parts[4]) if parts[4] not in ("-", "") else float("nan")
            rows.append((n, arm, sensor, z, dist, trip))
        elif line.startswith("SEARCHING"):
            trip += 1
            events.append((n, "search %d" % trip, "tab:blue"))
        elif line.startswith("PICK"):
            words = line.replace("|", " ").split()
            try:
                events.append((n, "PICK X %s" % words[2], "tab:green", float(words[2])))
            except (IndexError, ValueError):
                events.append((n, line, "tab:green"))
        elif line.startswith(("GRAB", "DONE", "ERROR", "STOP", "COLOUR")):
            events.append((n, line[:40], "tab:red" if line.startswith(("ERROR", "STOP")) else "black"))
    if not rows:
        print("no sensor lines in %s" % path)
        return
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(13, 8), gridspec_kw={"height_ratios": [3, 2]})
    for name, (left, right), colour in (("STORE", g["STORAGE_AREA"], "tab:green"),
                                        ("DISPOSE", g["DISPOSAL_AREA"], "tab:blue"),
                                        ("MINE", g["MINING_AREA"], "tab:orange")):
        top.axvspan(left, right, color=colour, alpha=0.08)
        top.text((left + right) / 2, 0, name, ha="center", va="top", fontsize=9, color=colour)
    cmap = plt.get_cmap("viridis")
    trips = max(r[5] for r in rows) or 1
    for t in sorted(set(r[5] for r in rows)):
        sel = [r for r in rows if r[5] == t]
        top.plot([r[2] for r in sel], [r[4] for r in sel], ".", ms=3, color=cmap(t / float(trips)),
                 label="before first search" if t == 0 else "search %d and what follows" % t)
    for e in events:
        if len(e) == 4:
            top.axvline(e[3], color="tab:green", ls="--", lw=1)
            top.text(e[3], 40, e[1], rotation=90, va="top", ha="right", fontsize=8, color="tab:green")
    top.axhline(g["WALL_Z"] - g["CUBE_SIZE"] / 2 + g["GRAB_DISTANCE"], color="r", ls=":", lw=1,
                label="farther = empty floor for the detector")
    top.axhline(g["HOLD_DISTANCE"], color="k", ls=":", lw=1, label="closer = cube in the claw")
    top.set_xlim(min(g["MINING_AREA"][0], min(r[2] for r in rows)) - 10, 10)
    top.set_ylim(700, 0)
    top.set_xlabel("sensor field X [mm] (0 = bumper, right)")
    top.set_ylabel("distance [mm] (near = up)")
    top.legend(fontsize=7, loc="lower left")
    top.set_title(os.path.basename(path))
    bottom.plot([r[0] for r in rows], [r[1] for r in rows], "-", lw=1, label="arm (claw) X")
    bottom.set_xlabel("log line (time order)")
    bottom.set_ylabel("arm X [mm]")
    zax = bottom.twinx()
    zax.plot([r[0] for r in rows], [r[3] for r in rows], "-", lw=1, color="tab:purple", label="Z")
    zax.set_ylabel("Z [mm]", color="tab:purple")
    for e in events:
        bottom.axvline(e[0], color=e[2], lw=0.8, alpha=0.6)
        bottom.text(e[0], bottom.get_ylim()[1], e[1], rotation=90, va="top", ha="right", fontsize=7, color=e[2])
    fig.tight_layout()
    out = "%s.run.png" % os.path.splitext(path)[0]
    fig.savefig(out, dpi=120)
    print("run plot: " + out)


if __name__ == "__main__":
    log = sys.argv[1] if len(sys.argv) > 1 else record()
    explain(log)
    try:
        import matplotlib.pyplot as plt
        plt.show()
    except ImportError:
        pass
