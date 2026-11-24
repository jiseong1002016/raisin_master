import argparse
from pathlib import Path
import re
import subprocess


RAISIM_NEEDED = re.compile(r"\(NEEDED\).*\[(libraisim(?:_contact)?d?\.so(?:\.[^\]]+)?)\]")


def select_sdk(home):
    link = home / "raisim_ws/raisim2Lib"
    if not link.is_symlink():
        raise ValueError(f"{link} must be a symlink to a versioned RaiSim SDK")
    sdk = link.resolve(strict=True)
    match = re.fullmatch(r"raisim(\d+\.\d+\.\d+)", sdk.name)
    if sdk.parent != link.parent.resolve() or not match:
        raise ValueError(f"{link} must resolve to raisim<VERSION> in {link.parent}")
    version = match[1]
    for relative in (
        "raisim/lib/cmake/raisim/raisimConfig.cmake",
        f"raisim/lib/libraisim.so.{version}",
        f"raisim/lib/libraisim_contact.so.{version}",
        "rayrai/lib/cmake/rayrai/rayraiConfig.cmake",
        "rayrai/lib/librayrai.so",
    ):
        path = (link / relative).resolve()
        if not path.is_file() or not path.is_relative_to(sdk):
            raise ValueError(f"{link} does not contain its own RaiSim {version} file: {relative}")
    return link, version


def verify_binary_abi(prefixes, version):
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
            for name in RAISIM_NEEDED.findall(result.stdout):
                if not name.endswith(f".so.{version}"):
                    raise ValueError(
                        f"{path} needs {name}, but ~/raisim_ws/raisim2Lib selects {version}. "
                        "Use a Raisin binary release built for the selected SDK."
                    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path, nargs="+")
    args = parser.parse_args()
    try:
        link, version = select_sdk(Path.home())
        verify_binary_abi(args.prefix, version)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")
    print(link)
