"""Generate src/main.py (what the brain runs) from src/robot.py (what we edit).

    python3 tools/build.py mission      # competition: MISSION only
    python3 tools/build.py calibrate    # CALIBRATE + TEST (Left/Right at start-up)
    python3 tools/build.py cal-z        # one calibration routine only (smallest):
                                        #   cal-z cal-x cal-grip cal-cube cal-colour cal-teach cal-dist
    python3 tools/build.py test         # TEST only
    python3 tools/build.py check        # each motor moves a bit by itself, reports OK / NOT MOVING
    python3 tools/build.py all          # everything (probably too big for the brain)

Then Build and Download in VS Code as usual.

Why: the IQ2 brain compiles the program itself (MicroPython 1.13) and runs out
of memory on the full source. This keeps only what the chosen mode can reach:
  - docstrings and comments are dropped
  - top-level functions, classes and constants nothing reachable uses are dropped
The result is checked against the memory budget if `micropython` is installed
(brew install micropython): the old template, which ran on the brain, needed
about 59 KB of heap to compile.
"""
import ast
import os
import subprocess
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SRC = os.path.join(ROOT, "src", "robot.py")
OUT = os.path.join(ROOT, "src", "main.py")

TARGETS = {
    # modes in the build, entry points main() may reach, entry points to cut
    "mission": (("MISSION",), {"Mission", "startup"}, {"calibration_menu", "test_mode", "motor_check"}),
    "calibrate": (("CALIBRATE", "TEST"), {"calibration_menu", "test_mode"}, {"Mission", "startup", "motor_check"}),
    "all": (("MISSION", "CALIBRATE", "TEST"), {"Mission", "startup", "calibration_menu", "test_mode"}, set()),
}
# one small program per calibration routine: (build name, routine name in CAL_ROUTINES)
CAL_PARTS = (("cal-z", "Z scale"), ("cal-x", "X scale"), ("cal-grip", "Grip"), ("cal-cube", "Cube pose"),
             ("cal-colour", "Colour"), ("cal-teach", "Teach pts"), ("cal-dist", "Distance"),
             ("cal-rate", "Rates"))
for _name, _routine in CAL_PARTS:
    TARGETS[_name] = (("CALIBRATE",), {"calibration_menu"}, {"Mission", "startup", "test_mode", "motor_check"}, _routine)
TARGETS["test"] = (("TEST",), {"test_mode"}, {"Mission", "startup", "calibration_menu", "motor_check"})
TARGETS["check"] = (("CHECK",), {"motor_check"}, {"Mission", "startup", "calibration_menu", "test_mode"})
HEAP_BUDGET_KB = 59     # what the working template needed; the brain's real limit is a bit higher


def defined_names(node):
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
        return {node.name}
    if isinstance(node, ast.Assign):
        return {n.id for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)}
    return set()


def used_names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def strip_docstrings(tree):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.ClassDef, ast.Module)):
            body = node.body
            if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
                    and isinstance(body[0].value.value, str):
                body.pop(0)
                if not body:
                    body.append(ast.Pass())


def set_constant(tree, name, value):
    for node in tree.body:
        if isinstance(node, ast.Assign) and name in defined_names(node):
            node.value = ast.parse(repr(value), mode="eval").body


def keep_routines(tree, names):
    """Shrink CAL_ROUTINES to the given menu entries."""
    for node in tree.body:
        if isinstance(node, ast.Assign) and "CAL_ROUTINES" in defined_names(node):
            node.value.elts = [e for e in node.value.elts if e.elts[0].value in names]


def prune(tree, cut):
    """Keep top-level statements reachable from main() and from non-definition code."""
    defs = {}
    for node in tree.body:
        for name in defined_names(node):
            defs[name] = node
    roots = [n for n in tree.body if not defined_names(n)]      # imports, if __name__ ...
    roots.append(defs["main"])
    keep = set()
    todo = list(roots)
    while todo:
        node = todo.pop()
        if id(node) in keep:
            continue
        keep.add(id(node))
        for name in used_names(node) - cut:
            if name in defs:
                todo.append(defs[name])
    tree.body = [n for n in tree.body if id(n) in keep]


def literal(node):
    """True for a number/string/bool/None literal or a tuple of them."""
    if isinstance(node, ast.Constant):
        return True
    if isinstance(node, ast.UnaryOp) and isinstance(node.operand, ast.Constant):
        return True
    if isinstance(node, ast.Tuple):
        return all(literal(e) for e in node.elts)
    return False


class Fold(ast.NodeTransformer):
    """Evaluate arithmetic on literals at build time: CUBE_MM / 2 -> 37.5."""

    def generic_visit(self, node):
        node = super().generic_visit(node)
        if isinstance(node, (ast.BinOp, ast.UnaryOp)) and all(
                literal(c) for c in ast.iter_child_nodes(node) if isinstance(c, ast.expr)):
            try:
                value = eval(compile(ast.Expression(node), "<fold>", "eval"))
            except Exception:  # noqa: BLE001 - leave anything odd unfolded
                return node
            if isinstance(value, (int, float)) and value >= 0:
                return ast.copy_location(ast.Constant(value), node)
        return node


def inline_constants(tree, keep):
    """Replace uses of top-level literal constants by their value and drop the
    assignment: fewer globals, fewer names, less memory on the brain."""
    assigned = {}
    for node in ast.walk(tree):
        for t in getattr(node, "targets", []):
            for n in ast.walk(t):
                if isinstance(n, ast.Name):
                    assigned[n.id] = assigned.get(n.id, 0) + 1
        if isinstance(node, (ast.Global, ast.Nonlocal)):
            for name in node.names:
                assigned[name] = 99
    consts = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                and literal(node.value) and assigned.get(node.targets[0].id) == 1
                and node.targets[0].id not in keep):
            consts[node.targets[0].id] = node.value

    class Inline(ast.NodeTransformer):
        def visit_Name(self, node):
            if isinstance(node.ctx, ast.Load) and node.id in consts:
                return ast.copy_location(consts[node.id], node)
            return node

    tree.body = [n for n in tree.body
                 if not (isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                         and n.targets[0].id in consts)]
    Inline().visit(tree)
    Fold().visit(tree)
    ast.fix_missing_locations(tree)
    return len(consts)


def prune_methods(tree):
    """Drop methods whose name is never used as an attribute anywhere else.
    Conservative: a name used on any object keeps that method in every class.
    Dunder methods and mission states (s_*, called through getattr) stay."""
    while True:
        used = set()
        for cls in tree.body:
            for node in ast.walk(cls):
                if isinstance(node, ast.Attribute):
                    used.add(node.attr)
        removed = False
        for cls in tree.body:
            if not isinstance(cls, ast.ClassDef):
                continue
            body = [m for m in cls.body if not (
                isinstance(m, ast.FunctionDef) and m.name not in used
                and not m.name.startswith("__") and not m.name.startswith("s_"))]
            if len(body) != len(cls.body):
                cls.body = body or [ast.Pass()]
                removed = True
        if not removed:
            return


def fix_mode(tree, mode):
    """Single-mode build: select_mode() just returns the mode."""
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "select_mode":
            node.body = [ast.Return(ast.Constant(mode))]
    ast.fix_missing_locations(tree)


def mode_branches(tree, modes):
    """In main(), keep only the `if mode == ...` branches this build can run,
    and drop `if dashboard:` blocks when TEST is not in the build."""

    def branch_mode(test):
        if (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name) and test.left.id == "mode"
                and isinstance(test.comparators[0], ast.Constant)):
            return test.comparators[0].value
        return None

    class Cut(ast.NodeTransformer):
        def visit_If(self, node):
            self.generic_visit(node)
            if isinstance(node.test, ast.Name) and node.test.id == "dashboard" and "TEST" not in modes:
                return node.orelse or None
            m = branch_mode(node.test)
            if m is None:
                return node
            if m in modes:
                if len(modes) == 1:
                    return node.body            # the only mode: no test needed
                node.orelse = node.orelse if any(x != m for x in modes) else []
                return node
            return node.orelse or None          # this branch can't run in this build

    Cut().visit(tree)
    ast.fix_missing_locations(tree)


def heap_needed_kb(path):
    """Smallest MicroPython heap that compiles `path`, or None without micropython."""
    try:
        subprocess.run(["micropython", "-c", "1"], capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    folder, mod = os.path.dirname(path), os.path.basename(path)[:-3]
    code = "import sys\nsys.path.insert(0, %r)\ntry:\n    __import__(%r)\nexcept ImportError:\n    pass\n" % (folder, mod)
    lo, hi = 4, 1024
    while hi - lo > 1:
        mid = (lo + hi) // 2
        ok = subprocess.run(["micropython", "-X", "heapsize=%dk" % mid, "-c", code],
                            capture_output=True).returncode == 0
        lo, hi = (lo, mid) if ok else (mid, hi)
    return hi


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else ""
    if target not in TARGETS:
        sys.exit("usage: python3 tools/build.py [%s]" % "|".join(TARGETS))
    modes, _, cut = TARGETS[target][:3]
    with open(SRC) as f:
        tree = ast.parse(f.read())
    strip_docstrings(tree)
    set_constant(tree, "BUILD_MODES", modes)
    set_constant(tree, "MODE", modes[0])
    if len(TARGETS[target]) > 3:
        keep_routines(tree, {TARGETS[target][3]})
    inlined = inline_constants(tree, keep=set())
    if len(modes) == 1:
        fix_mode(tree, modes[0])
    mode_branches(tree, modes)
    prune(tree, cut)
    prune_methods(tree)
    code = ("# GENERATED by tools/build.py %s from src/robot.py - do not edit, edit robot.py\n" % target
            + ast.unparse(tree) + "\n")
    with open(OUT, "w") as f:
        f.write(code)
    print("wrote src/main.py: %s build, %d lines, %.1f KB, %d constants inlined" % (
        target, code.count("\n"), len(code) / 1024.0, inlined))

    with tempfile.TemporaryDirectory() as tmp:
        probe = os.path.join(tmp, "probe_main.py")
        with open(probe, "w") as f:
            f.write(code)
        need = heap_needed_kb(probe)
    if need is None:
        print("(install micropython to check the memory budget: brew install micropython)")
    elif need <= HEAP_BUDGET_KB:
        print("compile memory: %d KB (template that ran on the brain: %d KB) -> OK" % (need, HEAP_BUDGET_KB))
    else:
        print("compile memory: %d KB > %d KB of the template that ran -> MAY NOT FIT" % (need, HEAP_BUDGET_KB))


if __name__ == "__main__":
    main()
