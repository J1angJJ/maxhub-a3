#!/usr/bin/env python3
"""Inspect the installed carm SDK without connecting to the robot.

This is a safe first-step script: it only imports the module and prints
available public names so the next script can use the real SDK API instead of
guessing it.
"""

import importlib
import inspect
import sys


MODULE_NAME = "carm"


def main() -> int:
    print("MAXHUB A3 carm SDK inspection")
    print(f"Module: {MODULE_NAME}")
    print()

    try:
        module = importlib.import_module(MODULE_NAME)
    except Exception as exc:  # noqa: BLE001 - show import errors clearly.
        print(f"[FAIL] Could not import {MODULE_NAME}: {exc}")
        print("Run: conda activate maxhub-a3")
        print("Then check: python -m pip show carm")
        return 1

    print(f"[OK] Imported {MODULE_NAME}")
    print(f"File: {getattr(module, '__file__', '<unknown>')}")
    print(f"Version: {getattr(module, '__version__', '<unknown>')}")
    print()

    public_names = [name for name in dir(module) if not name.startswith("_")]
    print("Public names:")
    for name in public_names:
        value = getattr(module, name)
        kind = "class" if inspect.isclass(value) else "function" if inspect.isfunction(value) else type(value).__name__
        print(f"- {name}: {kind}")

    print()
    print("No robot connection was attempted.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
