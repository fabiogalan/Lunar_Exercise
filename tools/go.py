"""One command: build, download to the brain, start it, open the live view.

    python3 tools/go.py cal-z          # any build from tools/build.py: test, check, cal-z, ... mission
    python3 tools/go.py cal-z --no-plot

Put the robot in the start pose BEFORE running this: the program starts right
after the download. Close the VEX terminal in VS Code first (one port user).
Uses vexcom, the download tool that ships with the VEX VS Code extension, the
same way the extension's "Build and Download" + "Play" buttons do.
"""
import glob
import json
import os
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
VEXCOM = sorted(glob.glob(os.path.expanduser(
    "~/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/tools/vexcom/*/osx/vexcom")))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    target = sys.argv[1]
    if not VEXCOM:
        sys.exit("vexcom not found: is the VEX VS Code extension installed?")
    with open(os.path.join(ROOT, ".vscode", "vex_project_settings.json")) as f:
        project = json.load(f)["project"]

    if subprocess.run([sys.executable, os.path.join(ROOT, "tools", "build.py"), target]).returncode:
        sys.exit("build failed")
    print("downloading to slot %s and starting..." % project["slot"])
    r = subprocess.run([VEXCOM[-1], "--name", project["name"], "--slot", str(project["slot"]),
                        "--write", os.path.join(ROOT, "src", "main.py"), "--run"],
                       capture_output=True, text=True)
    if r.returncode:
        sys.exit("download failed (is the VS Code terminal holding the port?):\n" + r.stdout + r.stderr)
    print("running.")
    if "--no-plot" not in sys.argv:
        os.execv(sys.executable, [sys.executable, os.path.join(ROOT, "tools", "live_plot.py")])


if __name__ == "__main__":
    main()
