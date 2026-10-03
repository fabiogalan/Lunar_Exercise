"""Write a compact copy of src/main.py for the Brain: build/main.py (what the VEX extension downloads).

    python3 tools/ship.py        # run before every download

Removes comments, docstrings and blank lines (also inside exec blocks), indents with one space, and puts
every top-level function and class into its own exec(...), so the Brain compiles them one at a time instead
of the whole file at once (03.10: whole file 80 KB, MemoryError on the Brain). No hand-made exec blocks are
needed in src/main.py. Always edit src/main.py; build/main.py is overwritten every time.
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(ROOT, "src", "main.py")
TARGET = os.path.join(ROOT, "build", "main.py")


class Strip(ast.NodeTransformer):
    def strip_docstring(self, node):
        self.generic_visit(node)
        body = node.body
        if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
                and isinstance(body[0].value.value, str):
            node.body = body[1:] or [ast.Pass()]
        return node

    visit_Module = visit_ClassDef = visit_FunctionDef = strip_docstring

    def visit_Call(self, node):
        self.generic_visit(node)
        if isinstance(node.func, ast.Name) and node.func.id == "exec" and node.args \
                and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            node.args[0] = ast.Constant(compact(node.args[0].value))     # the code inside exec, too
        return node


def compact(code):
    text = ast.unparse(Strip().visit(ast.parse(code)))
    lines = []
    for line in text.splitlines():
        if line.strip():
            indent = len(line) - len(line.lstrip(" "))
            lines.append(" " * (indent // 4) + line.lstrip(" "))
    return "\n".join(lines) + "\n"


def split(code):
    """Each top-level def/class becomes exec('<its compact source>')."""
    tree = ast.parse(code)
    for i, node in enumerate(tree.body):
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            call = ast.Call(ast.Name("exec", ast.Load()), [ast.Constant(compact(ast.unparse(node)))], [])
            tree.body[i] = ast.Expr(call)
    return compact(ast.unparse(tree))


def main():
    source = open(SOURCE).read()
    out = split(compact(source))
    compile(out, TARGET, "exec")                     # fails here, not on the Brain, if something broke
    os.makedirs(os.path.dirname(TARGET), exist_ok=True)
    with open(TARGET, "w") as f:
        f.write(out)
    print("build/main.py: %d bytes (src/main.py %d) - now download in VS Code" % (len(out), len(source)))


if __name__ == "__main__":
    main()
