# EE maintenance sequence integration

Header: `src/raisin_raipal/ee_maintenance_sequence.hpp`, namespace `raisin::robot`.
Header-only C++17; standard library only, no hardware or sensor I/O. All calls
belong on the owning control thread; the component has no synchronization.

## Callable API

`EeMaintenanceSequence(Config)` takes two-axis `lower_limit_rad` and
`upper_limit_rad`, `tolerance_rad`, `stable_time`, `target_timeout`, and
`feedback_timeout`. Limits must be finite, ordered, and contain zero on both
axes; invalid configuration throws `std::invalid_argument`. Supply the actual
runtime limits in left/right order (`ee_01_left`, `ee_02_right`), not URDF constants.
Timeout must cover bridge slew travel plus settling and stable time.

- `start(Mode, const Readback&, TimePoint)` resets the session and checks initial feedback.
- `update(TimePoint, const Readback&)` advances at most one waypoint per call.
- `target()` returns `const std::array<double, 2>&` in radians.
- `status()` returns the `Status` enum; `done()` means `DONE` only.
- `failed()` reports feedback, drive, tracking, or clock failure.
- `commandValid()` allows target submission during CLOSE/TEST/HOLD and DONE.
- `cancel()` latches CANCELLED; `reset()` returns to IDLE. Both invalidate commands.

`Readback` contains arrays `position_rad`, `feedback_valid`, `feedback_time`,
and `drive_fault`. Times use `std::chrono::steady_clock`. Map bridge
`State.axes[i].position_rad` into position; validity requires fresh calibrated
feedback, not merely a numeric placeholder. Map nonzero `error_code` or
`(status_word & 0x0f) == 0x08` into fault, matching the existing bridge.
Also propagate a core/bridge fault through `drive_fault` or cancel the session.
Use the timestamp of the actual feedback, not the time a cached State was read.
The existing State contract does not carry per-axis feedback timestamps: the
owner must track freshness or explicitly invalidate readback when stale.

There is no tactile input or arm dependency. Bridge enable/qualification,
maintenance mode, tactile policy, command rejection, and safe stop remain
the owner's responsibility. On failure/cancellation stop submitting targets
and invoke the bridge's safe stop. `target()` retains its last value for
inspection even when `commandValid()` is false.

## Modes and execution

`CLOSE` commands exactly `(0,0)` and completes only after both actual positions
stay within tolerance for `stable_time`. Main can continue sending zero and
heartbeats while the arm advances, then call `start(Mode::TEST, readback, now)`.
`GET_READY` closes then stays in HOLD, monitoring feedback, faults and tracking
until cancelled/reset. HOLD is ready when `status() == Status::HOLD`, not `done()`.

`TEST` first closes stably, then issues eight partial-angle targets: each axis
half excursion, full excursion, zero with the other closed, then both together
and zero. Excursion follows the farther-from-zero limit but is capped at 0.2 rad
and clipped to that axis interval. Every target requires stable actual arrival.
This deliberately avoids full-range and Cartesian-grid sweeps; it is not a
geometric collision certificate or physical hardware qualification.

The core permits get ready only with both measured fingers already within
0.01 rad of zero and retains zero throughout arm motion. It does not open or
close an initially open EE as part of get ready.

Continuous travel between endpoints is supplied by the bridge's existing
per-axis velocity-limited slew. Submit/refresh the current target every control
tick so command watchdogs remain serviced; the sequence does not generate
commands, sequence IDs, enable requests, or heartbeats itself.

Each target has a fixed timeout from issue, including stable time; a late arrival
does not override expiry. HOLD refreshes its timeout while stably at zero and
times out persistent drift. Nonfinite, invalid, future-dated or stale feedback
fails immediately, as does a drive fault or backwards update time. Failures and
cancellation latch until start/reset. DONE is terminal and no longer monitors
health: main owns monitoring while holding zero after CLOSE/TEST completion.

## Standalone deterministic check

From the package root, without launching hardware or a full package build:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined \
  -I../raisin_raipal/src/raisin_raipal ee/test/ee_maintenance_sequence_test.cpp \
  -o /tmp/ee_maintenance_sequence_test
/tmp/ee_maintenance_sequence_test
```

The test uses synthetic times/readback, verifies stable closure with interrupted
arrival, HOLD and drift timeout, bounded partial targets and narrower intervals,
ordered independent/coupled motions, tracking timeout, per-axis stale/invalid/
future/nonfinite feedback and faults, cancellation, reset and backwards time.
