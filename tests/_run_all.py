"""
Minimal test runner used only because pytest isn't installable in this
sandbox (no PyPI access). Not part of the deliverable's normal workflow
-- once the user has network access, `pip install pytest` and
`pytest tests/` work directly against the same test files, since they're
plain function-based pytest-style tests with no special runner
dependency beyond bare asserts.
"""
import importlib
import os
import sys
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

MODULES = ["test_pid", "test_kinematics", "test_servo_mapping"]

total = 0
failed = 0

for modname in MODULES:
    mod = importlib.import_module(modname)
    test_funcs = [getattr(mod, name) for name in dir(mod)
                  if name.startswith("test_") and callable(getattr(mod, name))]
    print(f"\n=== {modname} ({len(test_funcs)} tests) ===")
    for fn in test_funcs:
        total += 1
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception:
            failed += 1
            print(f"  FAIL  {fn.__name__}")
            traceback.print_exc()

print(f"\n{total - failed}/{total} passed")
sys.exit(1 if failed else 0)
