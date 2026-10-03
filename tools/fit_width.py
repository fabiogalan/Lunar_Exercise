"""Fit cube_width(d) = CUBE_WIDTH_AT_0 + CUBE_WIDTH_PER_MM * d from calibration scans (docs/calibration.md).

    python3 tools/fit_width.py logs/cal-*.log              # linear fit (what src/main.py uses)
    python3 tools/fit_width.py --degree 2 logs/cal-*.log   # compare with a parabola

Cuts every scan into segments exactly like CubeDetector (SAMPLE_STEP, SMOOTH_COUNT, JUMP_MM from src/main.py),
treats every near segment wider than MIN_PIECE as ONE cube (so place single cubes only), fits the width against
the sensor reading and prints the residuals and the lines to paste into src/main.py.
"""
import os
import sys

import numpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))
from run import load_program  # noqa: E402

MIN_PIECE = 30.0        # shorter near segments are edge fragments, not cubes


def outbound_scans(path):
    """[(beam X, Z, reading)] per scan, from SEARCHING to the leftmost point (the way back is dropped)."""
    scans, cur = [], None
    for line in open(path, errors="replace"):
        parts = line.strip().split(",")
        if line.startswith("SEARCHING"):
            cur = []
            scans.append(cur)
        elif line.startswith(("PICK", "DONE", "ERROR")):
            cur = None
        elif cur is not None and len(parts) == 5 and parts[4] not in ("-", ""):
            try:
                cur.append((float(parts[2]), float(parts[3]), float(parts[4])))
            except ValueError:
                pass
    out = []
    for s in scans:
        if s:
            turn = min(range(len(s)), key=lambda i: s[i][0])
            out.append(s[:turn + 1])
    return out


def segments(g, scan):
    """[(right X, left X, reading = mean of the last SMOOTH_COUNT, grab Z)], like CubeDetector.
    A cube cut off by the scan start or the wall is left out: place calibration cubes away from both."""
    start_x = g["SEARCH_START_X"] - g["SENSOR_X_OFFSET"]
    held, last = [], None
    for (x0, z0, r), (x1, _, _) in zip(scan, scan[1:] + [(scan[-1][0] - 3.0, 0, 0)]):
        x = x0
        while x > x1:                       # unchanged readings are not printed: hold each until the next
            if x <= start_x + 0.5 and (last is None or last - x >= g["SAMPLE_STEP"]):
                held.append((x, z0, r))
                last = x
            x -= 1.0
    segs, cur = [], None
    for x, z0, r in held:
        if cur and abs(r - sum(cur[2]) / len(cur[2])) <= g["JUMP_MM"]:
            cur[1], cur[2] = x, (cur[2] + [r])[-g["SMOOTH_COUNT"]:]
            continue
        cur = [x, x, [r], z0]
        segs.append(cur)
    segs = segs[1:-1]                       # the first and last are cut by the scan start / the wall
    return [(a, b, sum(rs) / len(rs), z0 + sum(rs) / len(rs) - g["GRAB_DISTANCE"]) for a, b, rs, z0 in segs]


def main(argv):
    degree = 1
    if argv[:1] == ["--degree"]:
        degree, argv = int(argv[1]), argv[2:]
    if not argv:
        sys.exit(__doc__)
    g = load_program()
    a0, k, tol = g["CUBE_WIDTH_AT_0"], g["CUBE_WIDTH_PER_MM"], g["CUBE_WIDTH_TOL"]
    floor = g["WALL_Z"] - 5
    points = []
    print("current: cube_width(d) = %.1f + %.3f d, tolerance +-%.0f, JUMP_MM %.1f\n" % (a0, k, tol, g["JUMP_MM"]))
    print("%-28s %15s %7s %7s %7s  %s" % ("log", "X right..left", "width", "reading", "now err", "verdict now"))
    for path in argv:
        for n, scan in enumerate(outbound_scans(path), 1):
            for right, left, d, z in segments(g, scan):
                width = right - left
                if z >= floor or width < MIN_PIECE:
                    continue
                err = width - (a0 + k * d)
                verdict = "no cube" if err < -tol else "ONE CUBE" if err <= tol else "row"
                print("%-28s %7.0f..%-7.0f %7.0f %7.0f %+7.0f  %s"
                      % ("%s #%d" % (os.path.basename(path), n), right, left, width, d, err, verdict))
                if verdict != "row":           # rows (2+ cubes merged) do not belong in a one-cube fit
                    points.append((d, width))
    if len(points) <= degree:
        sys.exit("\nneed more than %d cube segments to fit" % degree)
    d, w = numpy.array(points).T
    coef = numpy.polyfit(d, w, degree)
    res = w - numpy.polyval(coef, d)
    print("\nfit over %d cubes, readings %.0f..%.0f mm:" % (len(d), d.min(), d.max()))
    print("  width = " + " + ".join("%.4g d^%d" % (c, degree - i) for i, c in enumerate(coef)).replace(" d^0", ""))
    print("  residuals: max |%.1f| mm, std %.1f mm" % (abs(res).max(), res.std()))
    if degree == 1:
        print("\npaste into src/main.py:")
        print("CUBE_WIDTH_AT_0 = %.1f" % coef[1])
        print("CUBE_WIDTH_PER_MM = %.3f" % coef[0])
        print("CUBE_WIDTH_TOL must stay above %.0f (3 x max residual) and below 47 (half a cube + gap)"
              % (3 * abs(res).max()))


if __name__ == "__main__":
    main(sys.argv[1:])
