# EE home and commissioning settings

For a battery-backed absolute multiturn encoder, set
`ee.persistent_multiturn_home: true` in the hardware profile. The real
`single_L_ee` and `dual_arm_left_ee` profiles select this behavior.

Set Home is an explicit calibration operation performed only at the known
physical zero. It saves encoder offsets to `ee.homing_file`; it must not be
repeated at an arbitrary startup posture. On restart, a valid saved file with
a matching configuration fingerprint restores home without rewriting the file.
Loss of communication invalidates measured positions and disables output, but
does not discard this saved calibration. Recovery still requires fresh feedback,
identity/firmware validation and a new explicit enable; restoring home never
automatically enables a motor.

Missing, malformed or mismatched home files remain unhomed. If the encoder's
multiturn memory is lost (for example after battery failure), or the mechanical
reference changes, restore the physical zero and explicitly recalibrate. The
persistence option does not detect battery health or prove that an encoder
retained its count. Profiles without this option retain session-local homing.

Qualified axes may optionally specify `command_velocity_rad_s` as a positive
target-slew limit no greater than `max_velocity_rad_s`. The latter remains the
measured overspeed fault threshold. Omitting the command limit preserves prior
behavior. These parameters do not themselves establish a safe mechanical speed,
and the runtime still requires separately specified torque limits and PD gains.
