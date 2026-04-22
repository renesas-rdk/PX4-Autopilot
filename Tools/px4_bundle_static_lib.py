#!/usr/bin/env python3
"""Merge multiple static archives into a single PX4 library."""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create libpx4.a from component archives")
    parser.add_argument("--output", required=True, help="Target archive to write")
    parser.add_argument("--ar", default=os.environ.get("AR", "ar"), help="Path to the ar tool")
    parser.add_argument("--ranlib", default=os.environ.get("RANLIB", "ranlib"), help="Path to the ranlib tool")
    parser.add_argument("inputs", nargs="+", help="Static archives to combine")
    return parser.parse_args()


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def run_ar_script(ar: str, output: Path, inputs: list[str]) -> None:
    script_lines = [f"create {output}"]
    script_lines.extend(f"addlib {archive}" for archive in inputs)
    script_lines.append("save")
    script_lines.append("end")

    subprocess.run(
        [ar, "-M"],
        input="\n".join(script_lines) + "\n",
        text=True,
        check=True,
    )


def run_ranlib(ranlib: str, output: Path) -> None:
    if not ranlib:
        return

    subprocess.run([ranlib, str(output)], check=True)


def main() -> int:
    args = parse_args()
    output = Path(args.output)
    inputs = [Path(item) for item in args.inputs]

    if not inputs:
        print("No input archives supplied", file=sys.stderr)
        return 1

    ensure_parent(output)

    if output.exists():
        output.unlink()

    run_ar_script(args.ar, output, [str(archive) for archive in inputs])
    run_ranlib(args.ranlib, output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
