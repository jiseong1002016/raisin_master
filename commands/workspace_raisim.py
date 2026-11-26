"""RaiSim SDK for a Linux RAIPAL workspace.

The delivered (OTA) Raisin binaries fix the RaiSim ABI: when they need one version, the workspace builds against
`~/raisim_ws/raisim<VERSION>`, so the machine default `~/raisim_ws/raisim2Lib` can follow newer releases. Only a
workspace whose binaries need no RaiSim yet falls back to `raisim2Lib`.
"""
import argparse
from pathlib import Path
import re
import subprocess


RAISIM_NEEDED = re.compile(r"\(NEEDED\).*\[(libraisim(?:_contact)?d?\.so(?:\.[^\]]+)?)\]")
VERSIONED = re.compile(r"libraisim(?:_contact)?d?\.so\.(\d+\.\d+\.\d+)")


def _binaries(prefixes):
    seen = set()
    for prefix in prefixes:
        for path in sorted((*prefix.glob("lib/*.so*"), *prefix.glob("bin/*"))):
            binary = path.resolve()
            if binary in seen or not binary.is_file():
                continue
            seen.add(binary)
            with binary.open("rb") as file:
                if file.read(4) != b"\x7fELF":
                    continue
            result = subprocess.run(["readelf", "-dW", str(binary)], check=True, capture_output=True, text=True)
            yield path, RAISIM_NEEDED.findall(result.stdout)


def required_version(prefixes):
    """The one RaiSim version the delivered binaries link, or None when none links RaiSim."""
    versions = {}
    for path, names in _binaries(prefixes):
        for name in names:
            match = VERSIONED.fullmatch(name)
            if not match:
                raise ValueError(f"{path} needs unversioned {name}; cannot tell which RaiSim SDK it was built for")
            versions.setdefault(match[1], path)
    if len(versions) > 1:
        raise ValueError("delivered binaries need different RaiSim versions: "
                         + ", ".join(f"{v} ({p})" for v, p in sorted(versions.items())))
    return next(iter(versions), None)


def _check_sdk(root, version):
    for relative in (
        "raisim/lib/cmake/raisim/raisimConfig.cmake",
        f"raisim/lib/libraisim.so.{version}",
        f"raisim/lib/libraisim_contact.so.{version}",
        "rayrai/lib/cmake/rayrai/rayraiConfig.cmake",
        "rayrai/lib/librayrai.so",
    ):
        path = (root / relative).resolve()
        if not path.is_file() or not path.is_relative_to(root.resolve()):
            raise ValueError(f"{root} does not contain its own RaiSim {version} file: {relative}")


def select_sdk(home, prefixes=()):
    """(sdk directory, version): the delivered binaries' version under ~/raisim_ws, else the raisim2Lib default."""
    workspace = home / "raisim_ws"
    version = required_version(prefixes)
    if version is not None:
        sdk = workspace / f"raisim{version}"
        if not sdk.is_dir():
            raise ValueError(f"delivered Raisin binaries need RaiSim {version}; install it as {sdk}")
        _check_sdk(sdk, version)
        return sdk, version
    link = workspace / "raisim2Lib"
    if not link.is_symlink():
        raise ValueError(f"{link} must be a symlink to a versioned RaiSim SDK")
    sdk = link.resolve(strict=True)
    match = re.fullmatch(r"raisim(\d+\.\d+\.\d+)", sdk.name)
    if sdk.parent != workspace.resolve() or not match:
        raise ValueError(f"{link} must resolve to raisim<VERSION> in {workspace}")
    _check_sdk(link, match[1])
    return link, match[1]


def verify_binary_abi(prefixes, version):
    for path, names in _binaries(prefixes):
        for name in names:
            if not name.endswith(f".so.{version}"):
                raise ValueError(f"{path} needs {name}, but the workspace RaiSim SDK is {version}. "
                                 "Use a Raisin binary release built for one RaiSim version.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path, nargs="+")
    args = parser.parse_args()
    try:
        sdk, version = select_sdk(Path.home(), args.prefix)
        verify_binary_abi(args.prefix, version)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")
    print(sdk)
