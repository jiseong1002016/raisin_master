# EE maintenance sequence integration

Header: `src/raisin_raipal/ee_maintenance_sequence.hpp`, namespace `raisin::robot`.
Header-only C++17; standard library only, no hardware or sensor I/O. All calls
belong on the owning control thread; the component has no synchronization.

## Callable API

`EeMaintenanceSequence(Config)` takes two-axis `lower_limit_rad` and
`upper_limit_rad`, `tolerance_rad`, `stable_time`, `target_timeout`, and
`feedback_timeout`. Limits must be finite, ordered, and contain zero on both
axes; invalid configuration throws `std::invalid_argument`. Supply the actual
runtime limits in `State.axes` left/right order, not raw URDF joint order.
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

For the L-mounted cross-linked mechanism, the owner supplies
`max_relative_angle_rad = 0.743` and selects the positive-opening state axis as
`first_test_axis`, for both real and nonreal backends. In real runtime state
order, URDF opening angles are `theta8 = -axes.right` and
`theta9 = axes.left`. Their existing individual ranges are [0, 0.96] rad and
their relative range is `0 <= theta9 - theta8 <= 0.743` rad. This upper bound
is the intersection of the four passive follower limits under the L
cross-linked map, not a new commissioning cap or YAML enable flag.

In wiring order, the first EE motor (the operator's actuator 8, state `left`)
opens first, then the second EE motor (actuator 9, state `right`) opens while
the first holds. Both then return to zero together. These wiring-order names
are opposite the URDF joint numbers above. Starting at zero, the three
endpoints in runtime `(left, right)` order are `(excursion, 0)`,
`(excursion, -excursion)`, `(0, 0)`; in positive-opening `(theta8, theta9)`
fractions they are `(0, 1), (1, 1), (0, 0)`.
Excursion respects both existing axis ranges, the relative range, and the
30-degree (pi/6 rad) partial-test amplitude. Opening segments change one axis;
the final segment returns both with the same fraction and preserves zero
relative angle. Every segment is feasible throughout, not only at its endpoints.
Closure from an open pose
alternates joint 8 closure and joint 9 closure within the same relative range.
Get Ready still holds both axes at zero during arm motion. Other mount types
retain their independent-axis sequence. These are commanded-path checks,
not proof of physical tracking, collision clearance or PD quality.

`CLOSE` finishes at `(0,0)` and completes only after both actual positions
stay within tolerance for `stable_time`. Main can continue sending zero and
heartbeats while the arm advances, then call `start(Mode::TEST, readback, now)`.
`GET_READY` closes then stays in HOLD, monitoring feedback, faults and tracking
until cancelled/reset. HOLD is ready when `status() == Status::HOLD`, not `done()`.

For uncoupled mounts, `TEST` closes stably, then issues eight partial-angle targets: each axis
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
ordered independent/coupled motions, relative-angle-feasible waypoints and
interpolated segments in both runtime axis orders, staged closure from the
full-open pose, narrow axis intervals, tracking timeout, per-axis stale/invalid/
future/nonfinite feedback and faults, cancellation, reset and backwards time.
