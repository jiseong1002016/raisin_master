#!/usr/bin/env python3
"""Bind an existing licensed AnyGrasp environment without modifying its SDK."""
import argparse
import json
import os
from pathlib import Path
import shlex


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("sdk-dir", "checkpoint", "python", "site-packages", "libcrypto", "openblas", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    for name in ("sdk_dir", "checkpoint", "python", "site_packages", "libcrypto", "openblas"):
        path = getattr(args, name).expanduser().absolute()
        if name != "python":
            path = path.resolve(strict=True)
        directory = name in ("sdk_dir", "site_packages")
        if directory != path.is_dir():
            parser.error(f"--{name.replace('_', '-')} has the wrong file type")
        setattr(args, name, path)
    if not os.access(args.python, os.X_OK) or not (args.sdk_dir / "license").is_dir():
        parser.error("an executable Python and this machine's SDK license directory are required")
    output = args.output.expanduser().absolute()
    output.mkdir(parents=True, exist_ok=False)
    libraries = output / "lib"
    libraries.mkdir()
    for name, source in (("libcrypto.so.1.1", args.libcrypto), ("libopenblas.so.0", args.openblas)):
        (libraries / name).symlink_to(source)
    python = output / "python"
    python.write_text("#!/bin/sh\nexec env "
                      f"PYTHONPATH={shlex.quote(str(args.site_packages))}\"${{PYTHONPATH:+:$PYTHONPATH}}\" "
                      f"LD_LIBRARY_PATH={shlex.quote(str(libraries))}\"${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}\" "
                      "OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 "
                      f"{shlex.quote(str(args.python))} \"$@\"\n")
    python.chmod(0o755)
    config = {"sdk_dir": str(args.sdk_dir), "checkpoint_path": str(args.checkpoint),
              "python_executable": str(python)}
    (output / "anygrasp.json").write_text(json.dumps(config, indent=2) + "\n")
    print(json.dumps(config))


if __name__ == "__main__":
    main()
