# EE plant and articulated operator display

From `raisin_master/src/raisin_plugin`, regenerate a source-derived,
fixed-pose visual bundle with:

```sh
python3 ee/raisin_raipal_ee/tools/generate_passive_right_ee.py \
  --output-dir /tmp/raisin_tactile_live_20261003/urdf_stage --side L
python3 ee/raisin_raipal_ee/test/passive_visual_generator_test.py
```

The generator uses `ee_visual.urdf`, its CAD meshes, and the selected canonical mount in
`ee_mounts.yaml`; it exports the module through `resource/module-exporter.py`
and composes the full model through the existing `urdf-writer.py`.
For `--side L`, the staged module list keeps the arm at 14 movable joints and
adds the EE visual after `raipal_L`. The generated side-specific tip name is a zero-offset alias
at the EE root so `RobotConfig` can resolve its configured frame name. It is
not a calibrated fingertip or TCP.

The left mount removes the earlier 180-degree correction, which compensated an
unhomed L7 pose rather than a mounting error. Its original roll is -90 degrees;
the separate exact +30-degree hardware phase compensation remains, giving -60
degrees. Restore -90 degrees only after repairing the mechanical mounting phase.
The arm-control plant retains 7/14 movable coordinates; its EE module is fixed
so existing arm controllers keep their coordinate contract. The companion
`*_display.urdf` and `*_display.xml` contain the complete robot with eight EE
revolute joints and six canonical mimic relationships. RobotWindow loads this
single articulated model, including the left D405 visual. Its dimensions are
15/22; named arm updates preserve the additional EE coordinates.
The stage manifest records input and output hashes plus model checks.

Normal CMake configuration runs the generator into
`${CMAKE_CURRENT_BINARY_DIR}/passive_ee_stage`; `cmake --install` installs the
staged right and left EE modules, CAD meshes, preview XMLs, composed URDFs, and URDF-side meshes.
Build outside the source tree so generated resources remain in the binary dir.

Profile selection starts with packaged defaults, then uses
`hardware_profile` from `~/.raisin/raisin_raipal/params.yaml` when present; an
optional `--profile NAME` overrides it for this process only and is never saved
to YAML. With neither user setting, packaged `default_profile` is `dual_arm`.
The selected profile derives the expected slaves and EE communication, module,
XML, and GUI state. `dual_arm_right_ee` preserves the right mount, while
`dual_arm_left_ee` selects the canonical left mount. Both expect 18 slaves and
enable EE communication; actuation remains a separate gate and is never enabled
automatically. Use `dual_arm_left_ee` for the left grasp setup.
Its physical SOEM order is hubs 1–2, arm drives 3–16, and EE drives 17–18,
matching the Railab scan on 2026-10-22. The right-EE profile instead places
EE drives at 10–11 between the two arms. `single_L_ee` requires an actual
11-slave bus (two hubs, seven arm drives, two EE drives); it does not hide
the R-arm drives on an 18-slave bus. Select `dual_arm_left_ee` when both
arms and a left-mounted EE are connected.
A matching real home file seeds encoder offsets, but each new EtherCAT session
starts unhomed and requires explicit `set_home`; feedback invalidation or staleness
clears that session's home. A normal EE stop retains home while both feedback
streams stay fresh. The mock backend still restores its matching software home on
restart.

RobotWindow maps two valid drive output angles to eight display coordinates,
not eight measured encoder axes. Position feedback is independent of permission
to actuate: schema 3 publishes `position_valid` separately from
`calibrated_valid`. Missing or stale position data freezes finger coordinates;
the fixed EE mount still follows the arm.

The real left-mounted mechanism uses the cross-linked map, not the independent
left/right-finger approximation used by the mock preview. On the measured
dual-arm left-EE bus, state axis `left` (slave 17) is physical joint 9 and state
axis `right` (slave 18) is physical joint 8. With A=joint8 and B=joint9, the
display coordinates in `kPreviewJointNames` order are
`[A, B, -A-B, A+B, -B, -A, A+B, -A-B]`. Thus A=0 and B>0 opens both sides
symmetrically while keeping the pads parallel. This follows the crosswire
kinematics in grasp_iteration's RaipalArmInterface; the 2026-10-22 physical
reference is `01_temp/EE_9-jt_max.jpg`. Encoder zeros, motor signs, and reducer
conversion are not changed by this display-only mapping. No angles are silently
clipped to simulation limits. Mock and nonreal policy behavior is unchanged.

Both physical reducers are 68:1. URDF limits describe output radians, not motor
radians. Joint 8's `ee_01_left` limit is [-0.96, 0.132], intersected with the
[-0.96, 0] range of `ee_02_left`; joint 9's `ee_02_right` range is [0, 0.96].
The real L-mounted state axes are cross-linked: `left` takes joint 9's [0, 0.96],
and `right` takes joint 8's [-0.96, 0]. This fixes the former reversed 0.2-rad
commissioning bounds without changing encoder directions or saved home.
These driven-coordinate limits alone do not qualify coupled-link clearance.
Convert motor counts exactly once (0.96 output rad is 65.28 motor rad):
`q = direction * (count - position_reference_count) * 2*pi / (CPR * 68)`.
`position_reference_count` is the raw count at the URDF zero, not the count at
node startup. Motor polarity and zero must be measured or restored from a
verified calibration; fixture signs are not hardware calibration.

For position-only real feedback, set `ee.position_feedback_calibrated: true`
and provide each axis's explicit `gear_ratio`, `direction`, and
`position_reference_count` together with the existing encoder configuration.
This never enables motion. The packaged flag remains false until those values
are known; actuation and homing requirements are unchanged.

The real L+EE profiles enable `ee.position_homing_enabled` and store the two
encoder offsets separately in `homing_ee_L.conf`, relative to the node working
directory. With the arm in MOTOR_READY and both fingers at URDF zero (0, 0),
RobotWindow's **Set EE Home** calls `set_home_with_type` with `ee`. This samples
both encoders without changing arm offsets or enabling either EE drive. The
ordinary Main/Portable Set Home also includes the EE when an EE profile is
selected; numeric partial-arm requests still affect only those arm axes.
Tactile and D405 are not needed to record an encoder zero. Fresh validated PDO
feedback and fault-free drives are required. A new real session or lost feedback
requires an explicit Set Home again; a saved file alone never enables motion.

## SDK selection and Eigen compatibility

Source `scripts/raisim_sdk_env.sh` from the RAIPAL package with the selected
SDK symlink before configuring a build. On Railab, the project-local selector is
`/home/raipal/raisin_master/runtime/raisim2Lib`; its versioned SDK lives outside
the experiment isolation tree. Keep previous SDK installations intact and rebuild
dependencies against the selected version rather than aliasing an old SONAME to
a new library. The GUI FFmpeg transport must use the same RaiSim version as the GUI.

The focused EE build uses the same `RAISIN_MARCH` default as the master project:
`x86-64-v3` on x86 and `armv8.2-a+crypto+fp16+dotprod` on ARM. Match this setting
to the dependency SDK because Eigen values cross shared-library boundaries.
Mixing the x86 SDK's AVX alignment with a baseline SSE build caused a reproducible
invalid free during safety-controller initialization, including in nonreal mode.

The focused EE install also deploys `raisin_dongdrive` runtime configurations
from the workspace source or supplied SDK prefixes. Linking its static library
alone is insufficient: the real node loads the motor YAML files during Turn on.
`ee_dongdrive_config_install` checks these files in a fresh install prefix.
