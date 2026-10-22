# RAIPAL Linux workspace

Robot-specific defaults and user overrides follow
[robot configurations](robot_configurations.md); do not mix L+EE settings into
the original dual-arm `default` configuration.

The maintained CMake profile builds from `src/` into `cmake-build-release/` and
installs into `install/`. Runtime wrappers and isolation checkouts are not build
inputs. Installed executables load installed plugins, not workspace build outputs.

## SDK selection

Keep versioned SDK directories and select one through `raisim2Lib`. Do not copy
RaiSim binaries into the workspace install or remove its other dependencies.
The RAIPAL profile requires RaiSim 2.9.4, including its matching rayrai package.

Local uses `~/raisim_ws/raisim2Lib -> raisim2Lib_v294`; the workspace link points
to that default link. Railab uses:

```
~/raisin_master/raisim2Lib
  -> /home/raipal/workspace/jiseong/raisim_sdks/raisim2Lib
  -> raisim2Lib_v294
```

The existing 1.1.8 SDK remains available separately. Changing the selected SDK
requires a fresh configure/build/install and ABI validation, not just a relaunch.

## Source inputs

The validated robot configuration uses these package revisions on both machines:

- `src/raisin_raipal`: `4399e9c8fe712dff2fbe9da892b27e8969f05b83`.
- `src/raipal_controller`: `eca785fe9bf638cc6627e1e00d765614c8c22bee`.
- `src/raisin_gui`: `ecc7db3bc348d412c4b59c30072efd7a173dc1fc`.
- The nested `RAIPAL-URDF`: `4b313b7e81733dab29cf1855ec93f26c6a8f10a3`.

`cmake/raipal_workspace.cmake` owns package order and install rules. In addition
to the device, robot, GUI and controller repositories, it requires:

- `src/raisin`: `https://github.com/raionrobotics/raisin.git`,
  `acf94715c267523e1c5f0f38305aa191ae294488`.
- `src/raisin_robot_packages`:
  `https://github.com/raionrobotics/raisin_robot_packages.git`,
  `ac1d14637830555fa09f2961d4f9b540c331e393`.
- `src/raisin_third_party_gui`:
  `https://github.com/raionrobotics/raisin_third_party_gui.git`,
  `184c2a93df68a957a8d0e171db769c58835746f6`; this profile rebuilds its node editor
  against the selected SDK ImGui rather than a second installed ImGui header.

Preserve the working-tree patches to these repositories. The B0 controller uses
headers from the explicitly configured `GRASP_ITERATION_ROOT`; it does not use an
isolation copy. Existing third-party packages in `install/` remain dependencies.
Generated interface headers under `generated/include/` must already be present;
`--cmake-only` deliberately does not regenerate or remove them.

Keep source-owned headers out of `generated/include/`: an old hand-copied header
there can shadow the maintained source. Migration backups retain any such
overlays; the source package, not a generated-directory copy, owns those headers.

## Configure, build and install

Stop the node and GUI before installing. On Railab:

```bash
cd ~/raisin_master
./raisin setup --cmake-only
cmake --preset raipal-release \
  -DGRASP_ITERATION_ROOT="$HOME/workspace/jiseong/grasp_iteration"
cmake --build --preset raipal-release
cmake --install cmake-build-release
ctest --test-dir cmake-build-release --output-on-failure
```

Run the GUI tests from the desktop session. An SSH shell must inherit its
`DISPLAY`, `XAUTHORITY` and `XDG_RUNTIME_DIR`; on the validated Railab desktop
these were `:1`, `/run/user/1000/gdm/Xauthority` and `/run/user/1000`.

`--cmake-only` updates the generated root CMake without deleting the existing
install, external dependencies, generated headers or user settings. Plain
`raisin setup` retains its upstream clean/deploy behavior and is not the command
for this migration. Do not edit `CMakeCache.txt` to move an existing build. Archive
an obsolete build directory and configure the prescribed directory afresh.

Installed parameter defaults come from source. Hardware profiles, motor home
files and per-user `~/.raisin/` overrides are separate from binary installation;
back them up before a migration. This profile does not change EE gains, limits,
joint conventions or home calibration.

## Run the installed products

```bash
cd ~/raisin_master
source ./ld_prefix_path.sh
./install/bin/raisin_raipal_node --profile single_L_ee_nonreal
```

In a second terminal:

```bash
cd ~/raisin_master
source ./ld_prefix_path.sh
./install/bin/raisin_gui
```

Omitting `real` selects simulation; `nonreal` is not a positional CLI argument.
The explicit mock profile avoids the real motor bus. A separately coordinated
real launch uses `sudo env HOME="$HOME" LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
./install/bin/raisin_raipal_node real`; it uses the selected
installed hardware profile. Real motor activation and motion require a separate
safety check and are not part of build/install validation.

## Validation

Inspect `ldd` for the node, GUI, controllers and EE/D405 plugins. Inspect
`/proc/<pid>/maps` after startup as well: dynamically loaded windows/plugins do
not all appear in the executable's `ldd` output. Expect `libraisim.so.2.9.4` from
the selected versioned SDK and Raisin products from `install/`. Reject 1.1.8,
retired `runtime/grasp_294` or `runtime/gui_294`, isolation products and deleted
mapped files. Source-only nonreal checks must not access the motor bus or camera.
Use the sourced environment when inspecting standalone plugin libraries as well;
some SDK-private transitive dependencies need the SDK library directories in
`LD_LIBRARY_PATH`. Do not rewrite vendor binaries to change their RPATH.

The robot and Developer GUI profile tests inject nonzero measured EE states;
a fresh neutral mock display alone does not establish real measured-joint motion.
