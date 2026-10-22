# RAIPAL plugins

This source collection is versioned in the `raisin_master` repository.
`release/install/raisin_plugin` contains SDK build products, not editable sources.

- `ee/raisin_raipal_ee`: EE plugin, command contract, mechanics, resources, and tests.
- `ee/raisin_gui_raipal_ee_window`: EE GUI plugin.
- `ee/test`: offline EE and robot integration checks.
- `raisin_d405_plugin`: real/nonreal camera plugin and its resources.

The robot-specific bridge and maintenance sequence remain in
`../raisin_raipal/src/raisin_raipal`. The node retains ownership of the shared
EtherCAT bus. This relocation does not change control laws, calibration,
protection limits, package names, or network service names.

The RAIPAL package adds the EE plugin targets from this collection. Its focused
SDK build entrypoint is `ee/CMakeLists.txt`; supply `RAISIN_MASTER_WORKSPACE`,
`RAISIN_SDK_PREFIXES`, dependency prefixes, and `CMAKE_INSTALL_PREFIX` as before.
The D405 plugin can be built independently with `raisin_d405_plugin/scripts/build.sh`.
The collection's root `CMakeLists.txt` is the normal workspace discovery boundary;
it builds EE, EE GUI, and D405 without exposing the focused SDK staging project
as a second workspace package.

Run the resource-generation regression without a robot:

```bash
python3 src/raisin_plugin/ee/raisin_raipal_ee/test/passive_visual_generator_test.py
```

From the workspace root, the dependency-free maintenance check is:

```bash
c++ -std=c++20 -UNDEBUG -Isrc/raisin_raipal/src/raisin_raipal \
  src/raisin_plugin/ee/test/ee_maintenance_sequence_test.cpp \
  -o /tmp/ee_maintenance_sequence_test
/tmp/ee_maintenance_sequence_test
```
