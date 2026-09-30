"""Live view: arm, distance beam, picked cube locations, distance over time.

    pip3 install pyserial matplotlib
    python3 tools/live_plot.py                         # auto: listens on all brain USB ports
    python3 tools/live_plot.py --port /dev/cu.usbmodem1103
    python3 tools/live_plot.py --file sim.log          # replay a saved log / sim output

Works in every MODE (mission, calibration, test). It replaces the VEX terminal:
close the terminal in VS Code first, only one program can hold the port.
Everything that is not telemetry (calibration results, CONFIG lines to paste,
warnings) is printed here and everything is saved to logs/<date-time>.log.

Telemetry lines from src/main.py:
    R,x,z,d[,hit_x,hit_z]   live robot line, d = -1: nothing seen
    C,x,z,colour            cube identified
    E,text                 status, calibration results, or error
"""
import collections
import glob
import os
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

HISTORY_S = 30.0

cubes = []                                  # (x, z, colour)
robot = {"x": 0.0, "z": 0.0, "d": None, "hit": None}
dist_hist = collections.deque(maxlen=2000)  # (t, d)
status = [""]
lock = threading.Lock()
log_file = [None]
t0 = time.time()


def handle(line, echo=True):
    line = line.strip()
    if not line:
        return
    if log_file[0]:
        log_file[0].write(line + "\n")
        log_file[0].flush()
    parts = line.split(",")
    kind = parts[0]
    try:
        with lock:
            if kind == "R":
                robot["x"], robot["z"] = float(parts[1]), float(parts[2])
                d = float(parts[3])
                robot["d"] = None if d < 0 else d
                robot["hit"] = (float(parts[4]), float(parts[5])) if len(parts) >= 6 else None
                dist_hist.append((time.time() - t0, robot["d"]))
                return
            if kind == "C":
                cubes.append((float(parts[1]), float(parts[2]), parts[3]))
            elif kind == "E":
                status[0] = line[2:]
    except (ValueError, IndexError):
        pass
    if echo:
        print(line)     # calibration results, CONFIG lines, events, screen text


def read_serial(port):
    import serial   # checked in main()
    try:
        with serial.Serial(port, 115200, timeout=1) as s:
            print("listening on " + port)
            while True:
                raw = s.readline()
                text = raw.decode("utf-8", "replace")
                if text.strip() and all(31 < ord(c) < 127 or c in "\r\n\t" for c in text):
                    handle(text)    # binary chatter on the system port is dropped
    except Exception as e:  # noqa: BLE001 - one port failing must not stop the other
        print("%s: %s" % (port, e))


def draw(ax_map, ax_d):
    with lock:
        cs = list(cubes)
        rx, rz, d, hit = robot["x"], robot["z"], robot["d"], robot["hit"]
        hist = list(dist_hist)
        st = status[0]

    ax_map.clear()
    for x, z, colour in cs:
        ax_map.plot(x, z, "s", markersize=9, markeredgecolor="k",
                    color={"red": "tab:red", "green": "tab:green", "blue": "tab:blue"}.get(colour, "k"))
    ax_map.plot([rx, rx], [0, rz], color="tab:orange", lw=5, label="arm")
    if hit is not None:
        ax_map.plot([hit[0], hit[0]], [rz, hit[1]], ":", color="tab:purple")
        ax_map.plot(hit[0], hit[1], "*", color="tab:purple", markersize=14, label="beam hit")
    ax_map.set_xlabel("X [mm]: home at 0, left is negative")
    ax_map.set_ylabel("Z into field [mm]")
    ax_map.invert_yaxis()
    ax_map.set_title("X %.0f  Z %.0f  d %s   |   %s" % (rx, rz, "-" if d is None else "%.0f mm" % d, st))
    ax_map.legend(loc="lower right", fontsize=8)

    ax_d.clear()
    if hist:
        now = hist[-1][0]
        pts = [(t - now, v) for t, v in hist if t > now - HISTORY_S and v is not None]
        if pts:
            ax_d.plot([p[0] for p in pts], [p[1] for p in pts], color="tab:purple")
    ax_d.set_xlim(-HISTORY_S, 0)
    ax_d.set_xlabel("time [s]")
    ax_d.set_ylabel("distance [mm]")
    ax_d.grid(alpha=0.3)


def main():
    fig, (ax_map, ax_d) = plt.subplots(2, 1, figsize=(11, 7), gridspec_kw={"height_ratios": [2, 1]})
    fig.tight_layout(pad=2.5)
    if "--file" in sys.argv:
        with open(sys.argv[sys.argv.index("--file") + 1]) as f:
            for line in f:
                handle(line, echo=False)
        draw(ax_map, ax_d)
        plt.show()
        return

    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial missing: %s -m pip install pyserial" % sys.executable)
    os.makedirs("logs", exist_ok=True)
    name = os.path.join("logs", time.strftime("%Y%m%d-%H%M%S") + ".log")
    log_file[0] = open(name, "w")
    print("saving everything to " + name)
    if "--port" in sys.argv:
        ports = [sys.argv[sys.argv.index("--port") + 1]]
    else:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if not ports:
            sys.exit("no /dev/cu.usbmodem* port: is the brain connected by USB?")
    for port in ports:      # the brain has two ports; listen on both, junk is filtered
        threading.Thread(target=read_serial, args=(port,), daemon=True).start()
    anim = FuncAnimation(fig, lambda _: draw(ax_map, ax_d), interval=200, cache_frame_data=False)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    main()
