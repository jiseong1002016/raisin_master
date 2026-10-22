# Raisin tactile USB integration

All tactile packages live in the `raisin_master` Git repository under
`src/raisin_plugin/tactile/`: `raisin_tactile_sensor`, `raisin_tactile_plugin`, and
`raisin_gui_tactile_window`.

The USB backend opens one serial stream in a receive-only child process. Each
complete 12-packet set becomes one 71×41 map. The configured map order is right
then left; with one connected sensor, `stream_sensor_count: 1` routes every map
to right. With two sensors, set it to `2` and maps alternate right, left. The
Raisin plugin supervises the process; core-owned `ReceiverV2` instances expose
`tactile/left` and `tactile/right`; the GUI shows both maps and writes replayable
frame files. Nothing in this path commands a motor.

## Data contract

- USB producer topics: `tactile/v2/left`, `tactile/v2/right`, and per-side
  `tactile/v2/status/{left,right}`. Baseline requests use
  `tactile/v2/command` with `capture_baseline:left`, `:right`, or `:both`.
- Core registry keys: `tactile/left` and `tactile/right`, each a
  `raisin::tactile::ReceiverV2`.
- Optional shadow pair events: `tactile/v2/shadow/event`. The plugin checks the
  core Receivers on its 100 Hz rate and emits once when both sides have a new,
  baseline-valid frame within the configured stale and skew limits.
- Sensor frames keep signed raw code, per-side baseline, provisional force map,
  validity mask, source session, sequence, host receive timestamp, and
  calibration metadata. TX column 40 is invalid because the Qt reader marks it
  unscanned.
- The initial `0.001 N_estimate/code` conversion is provisional only. Replace
  it with per-sensor measured calibration before treating the values as force.
  Each source config has a TODO beside the coefficient.

**Stream order assumption:** the Qt 494-byte packet has a frame index but no
sensor ID; one set of 12 packets forms one 71×41 map. The user selected right
first, so two-sensor mode assigns complete maps right, left, right, left. A new
USB session starts at right. The physical two-sensor stream has not yet been
verified; confirm the order by pressing each pad separately before relying on
left/right identity. If a whole map or an unusable map-start packet disappears,
this protocol has no sensor ID or map counter to recover the phase.

## Configure a receive-only run

After installing the packages into the Raisin workspace you intend to use, set
the tactile plugin parameters in `config/raisin_tactile_plugin/config/params.yaml`:

```yaml
enabled: true
backend: usb
rate: 100
usb:
  port: /dev/serial/by-id/REPLACE_WITH_TACTILE_DEVICE
  stream_sensor_count: 1
  record_dir: /absolute/path/to/run/tactile
  topic_prefix: tactile/v2
  rate_hz: 30
  baud_rate: 115200
  frame_timeout_s: 0.5
  retry_s: 1.0
  poll_rate_hz: 100
shadow:
  enabled: true
```

Keep the configured `bilateral.stale_timeout_s` and `max_skew_s` with the run
manifest; the initial values are 0.15 s and 0.05 s. Enable
`tactile_bridge_enabled: true` in the RAIPAL parameters so the core registers
both Receivers. Keep the existing controller/motor activation path disabled
while validating tactile input.

Load `raisin_gui_tactile_window` in Raisin GUI. Its
`config/raisin_gui_tactile_window/config/params.yaml` must set `record_root` to
the same absolute directory as `usb.record_dir`. The window shows both maps,
port/session/sequence/FPS/age and parser counters, baseline/calibration state,
and bilateral skew. With two unloaded sensors, press **Capture both** once.
With one sensor, use **Capture right**; the left status remains disabled. With
two sensors, use **Capture both**. The GUI records per-side `.tfr2` frames under
a timestamped `frames_*` directory; the USB process creates a unique `usb_*`
subdirectory with one `stream.rawpkt` and a `run_manifest.json` settings snapshot.
Each frame file can be read with `raisin::tactile::readRecordedFrames()`.

When the second sensor is connected, change only `stream_sensor_count` from `1`
to `2`. The source already routes complete maps right first, then left. Confirm
that the controller emits a complete map for each pad in that order; the map
order cannot be inferred from the current packet fields alone.

For identity confirmation, press each pad separately and verify only the
intended left or right map changes. Unplug and reconnect the shared USB stream;
both logical sides must enter a new session and require fresh unloaded
baselines. Unloading/reloading the plugin must restart the USB process and
establish a fresh session. These physical checks are not yet complete. A
one-board Qt receive and direct USB-backend smoke were performed on the
development workstation, but those local records are not part of this
repository. They do not verify two-device identity, Raisin GUI recording/replay,
or physical reconnect.

## Build from a Raisin workspace

The packages live at `src/raisin_plugin/tactile` inside a Raisin workspace.
The workspace supplies `cmake/raisin_helper.cmake`, generated message headers,
and ABI-compatible Raisin packages. On a clean source workspace, run the normal
`raisin setup` message-generation step first; it writes the `.msg` headers to
`$RAISIN_WS/generated/include`. The tactile packages require
`raisin_plugin`, `raisin_network`, `raisin_robot`, `raisin_gui_base`, Eigen3,
and the dependencies exported by those packages. Install/build the Raisin SDK
for the target computer first. CMake prefixes and cached build directories from
another computer are not portable.

Use fresh build and install directories. The sensor package derives the normal
generated-header location from the workspace root. Set
`RAISIN_GENERATED_INCLUDE` only when using a separate generated-header tree.

```bash
RAISIN_WS="${RAISIN_WS:-$HOME/raisin_master}"
TACTILE_SRC="$RAISIN_WS/src/raisin_plugin/tactile"
TACTILE_BUILD="${TACTILE_BUILD:-$RAISIN_WS/.build/tactile}"
RAISIN_PREFIX="${RAISIN_PREFIX:-$RAISIN_WS/install}"
TACTILE_PREFIX="${TACTILE_PREFIX:-$TACTILE_BUILD/install}"

cmake -S "$TACTILE_SRC/raisin_tactile_plugin" -B "$TACTILE_BUILD/plugin" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$TACTILE_PREFIX" \
  -DCMAKE_PREFIX_PATH="$RAISIN_PREFIX"
cmake --build "$TACTILE_BUILD/plugin" -j2
cmake --install "$TACTILE_BUILD/plugin"
ctest --test-dir "$TACTILE_BUILD/plugin" \
  --output-on-failure

cmake -S "$TACTILE_SRC/raisin_gui_tactile_window" -B "$TACTILE_BUILD/gui" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$TACTILE_PREFIX" \
  -DCMAKE_PREFIX_PATH="$TACTILE_PREFIX;$RAISIN_PREFIX"
cmake --build "$TACTILE_BUILD/gui" -j2
cmake --install "$TACTILE_BUILD/gui"
ctest --test-dir "$TACTILE_BUILD/gui" --output-on-failure
```

CTest uses Qt-derived fixture bytes and one PTY, not physical sensors. The USB
backend test sends a right map followed by a left map on that one stream, and
also checks that one-sensor mode routes every map to right. The 100 Hz
shadow test verifies exactly one event per new 20 Hz pair and zero events for
duplicate, one-sided, stale, or future input. The process integration test
exercises right-first demultiplexing, both topic streams, independent baseline
capture, and stop/reload with new sessions. Keep test logs and hardware
acceptance records in a workspace-local evidence directory; they are not
required build inputs.

The `tactile_usb_plugin_lifecycle` test also loads the staged plugin through
its Raisin factory in real mode, calls `init()` and `advance()` at 100 Hz,
checks the single PTY startup command sequence, then destroys and recreates the
plugin and confirms a fresh USB run manifest. This verifies the plugin factory
and child-process lifecycle with PTYs; physical-device reconnect and a live
desktop display still require hardware and a running Raisin GUI.

The headless `tactile_gui_manager_load` test checks left-only and bilateral draw
data. If a fresh configure reports a missing package, include, or library path,
inspect the exported Raisin dependency named in the CMake error and install or
rebuild that dependency for the current computer before configuring into a new
build directory.

## Two independent USB boards

Leave `usb.port` empty and set distinct `usb.left_port` and `usb.right_port`
using stable `/dev/serial/by-id/` paths. Each stream carries one sensor map;
`stream_sensor_count` is effectively 1. The plugin reuses the existing USB
process per port, passing an explicit left/right identity. Only that side's
frame and status topics are published. Raw captures are separated under
`usb.record_dir/left` and `/right`. Shared-stream settings remain supported
when both new port fields are empty. Two PTY integration tests cover shared
and independent input, per-side baselines and disconnect isolation.
