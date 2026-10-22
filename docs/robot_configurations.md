# Robot configurations

Robot-dependent parameters belong to their owning package, under
`config/default/` or `config/L_ee/`. Shared C++ implementations, meshes and motor
drivers are not duplicated. The GUI receives the selected robot modules and DOF
from the node; it does not choose a separate robot configuration.

## Selection

`src/raisin_raipal/config/params.yaml` contains only the default profile selector.
The per-machine selector is `~/.raisin/raisin_raipal/params.yaml`. For L+EE:

```yaml
hardware_profile: single_L_ee
```

`hardware_profiles.yaml` indexes the configuration folders. `default` selects
the original dual-arm configuration, while `single_L_ee` selects `L_ee` with
seven arm joints, two EE axes, and no R-arm model. `single_L_ee_nonreal` uses
the same robot configuration with the mock backend. Existing explicit
`dual_arm_left_ee` and `dual_arm_right_ee` profiles retain their original bus
layouts for compatibility; neither is silently converted into a single arm.

The original dual-arm parameter file is preserved byte-for-byte under
`src/raisin_raipal/config/default/params.yaml`, from `raisin_raipal` commit
`a4132508f665444fd35bf92e9b158565b3de6d77`, blob
`0b65fa8df02618d060148195d7f83a6b33bbedc1`.

## Loading order

Node and controller parameters load these layers:

1. Installed `config/default/params.yaml` provides the shared baseline.
2. Installed `config/<configuration>/params.yaml` provides robot-specific values.
3. `~/.raisin/<package>/<configuration>/params.yaml` provides machine/user values.

The node then applies the selected hardware profile's bus mapping and EE
contract. Those authoritative records live in the corresponding configuration
folder, including the existing gains, limits, homing settings and qualification
flags. Moving them does not qualify or enable a previously disabled profile.

The flat user `params.yaml` is a selector only for the node. Flat controller
overrides, and another robot's user overrides, are not loaded. Back up old user
files before migrating the intended values into the selected subfolder. Home
calibration files stay in their existing locations and are not regenerated.

For example, set the L+EE EtherCAT interface in
`~/.raisin/raisin_raipal/L_ee/params.yaml`, not in the flat selector:

```yaml
ethercat_ifname: enx588694f8335b
```

Controller configuration folders follow the same rule. Original joint-test
parameters remain under `default`; `L_ee` adds `test_arm_sides: [L]`. Trajectory
catalogs and motion-editor catalogs use the selected configuration directory,
so an L-only robot is not offered the default dual-arm demonstrations. An
explicit `runtime_config_directory` remains an intentional external input.
An L+EE-only trajectory may provide its own `settings.yaml` without creating a
matching preset under `default`; its selected user override still applies.

Shared URDF modules and meshes remain in their canonical asset directories.
The selected hardware profile pins the matching scene and generated assembly;
copying a mesh library or controller implementation per robot is unnecessary.

## Adding another robot

Add `config/<name>/params.yaml` and `hardware_profiles.yaml` to `raisin_raipal`,
then register `<name>: <name>/hardware_profiles.yaml` in the index. Add only the
necessary parameter overrides and trajectory assets under each affected
controller's `config/<name>/`. A new hardware layout must also satisfy the
existing strict profile, PDO and GUI contracts; adding a folder does not bypass
those checks.

## Build and run

Build and install from the official workspace, with node and GUI stopped:

```bash
cd ~/raisin_master
cmake --preset raipal-release
cmake --build --preset raipal-release
cmake --install cmake-build-release
ctest --test-dir cmake-build-release --output-on-failure
```

Start a no-hardware L+EE node:

```bash
source ./ld_prefix_path.sh
./install/bin/raisin_raipal_node --profile single_L_ee_nonreal
```

For a separately coordinated real launch:

```bash
sudo env HOME="$HOME" LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
  ./install/bin/raisin_raipal_node real --profile single_L_ee
```

Start the GUI in another terminal with the same environment:

```bash
cd ~/raisin_master
source ./ld_prefix_path.sh
./install/bin/raisin_gui
```

The single-arm real profile requires eleven selected EtherCAT slaves: two hubs,
L1–L7 and two EE axes. It supports these explicitly declared physical layouts:

- Standalone eleven-slave bus: hubs 1–2, L at 3–9, EE at 10–11.
- Eighteen-slave bus: hubs 1–2, L at 10–16, EE at 17–18; R at 3–9 stays unused.

The extra layout is in `config/L_ee/hardware_profiles.yaml` under
`ethercat.additional_bus_layouts`. Every physical drive must be accounted for
exactly once; selected and inactive drives must match their identity/PDO pins.
Only selected drives enter the command-device list. Existing EtherCAT startup
zeroes every mapped output, including unused drives, and WKC still covers the
entire physical bus. Scan-to-SAFE_OP/OP count changes remain an error. This is
not an arbitrary “at least eleven” check and does not automatically enable EE
actuation. Saved home is matched to the resolved EE addresses, without changing
home files or their calibration values.

Node startup itself does not authorize Init Motor, Set Home, Joint Test,
Get Ready or motor motion.
