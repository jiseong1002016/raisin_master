# raisin_d405_plugin

Native Raisin D405 acquisition with the existing camera message and GUI transport.
Load the plugin through `create_with_owner_title_robot`; real mode requires one
matching USB D405. Nonreal mode marks every grasp snapshot `is_simulation: true`
and does not open a USB camera. Its default `simulation_source: plane` is the
deterministic transport fixture.
The capture thread runs independently of the plugin control loop.

For the grasp example, set `simulation_source: world`. Rays use the current
`worldSim_` collision geometry and the selected EE flange
(`simulation_ee_frame`, default `L_ee`), with the nominal mount below and
canonical side-specific flange-to-`ee_base` transform. The closest surface sets
depth; an empty scene produces invalid depth without a plane fallback.
`simulation_ray_stride: 16` casts 40×30 rays and expands them to the standard
640×480 image. One coarse row holds the simulation mutex at a time; image
publication and AV1 encoding run after releasing it. The camera pose is sampled
once per frame while scene rows may sample successive physics steps. This is a
coarse collision depth view, without RealSense noise, distortion or measured
calibration. The serial is `simulation_world`, and snapshots identify
`simulation_source` explicitly. Real acquisition ignores these simulation settings.

Run `bash scripts/build.sh` from this package. It builds with one compiler process
and installs into the workspace prefix. `D405_BUILD_DIR` and `D405_INSTALL_DIR`
can select separate build/install directories.

Default prefix `railab_raipal/d405` exposes:

- `/depth`: `sensor_msgs::Image`, `32FC1`, depth in meters.
- `/depth/camera_info`: `sensor_msgs::CameraInfo`.
- `/depth/get_camera_info`: the existing `GetCameraInfo` service.
- `/depth_compressed`: existing `raisin_interfaces::msg::FfmpegPacket` AV1 GUI transport, encoded when subscribed.
- `/grasp_input`: `std_msgs::String` containing `raisin_d405_grasp_input_v1` JSON.
- `/status`: capture errors; failed real acquisition stops without synthetic fallback.

The grasp snapshot contains `valid`, `is_simulation`, `sequence`,
`stamp_monotonic_ns`, `serial`, `frame_id`, `roi_depth_xywh`, `valid_pixels`,
`depth_median_m`, `point_camera_m`, `points_camera_m`, `points_pixel_stride`, and `depth_intrinsics`
(`width,height,fx,fy,ppx,ppy,model,coeffs`). Invalid ROI depth produces a null point.
The timestamp is host receive time in `CLOCK_MONOTONIC`, not a hardware exposure
timestamp. The ROI identifies a visible surface; it does not detect an object or
predict a learned grasp. The controller applies its camera extrinsic and explicit
surface-to-grasp offset while the arm is held still.

Only the pixels a ray actually sampled are reported in `points_camera_m`.
The world depth image expands one coarse ray's depth over its whole
`simulation_ray_stride` block, so the other pixels of a block carry a
neighbour's depth; deprojecting one of them at its own pixel centre places
the point off the surface it belongs to. A block that straddles an object's
silhouette therefore contributed points that lie inside the object's own
bounds but off its surface, and the source-alignment registration fitted
those instead of the object. The plugin deprojects at the centre of the sampled block in world simulation;
real and plane streams keep their actual pixel coordinates. No sample mask is
needed. The published depth image is unchanged.

`points_camera_m` contains at most 1,200 strided valid ROI samples in optical
coordinates. The source-aligned grasp example uses these measured points with
an explicitly selected source mesh and orientation prior to estimate translation.
The source scene launcher projects the qualified pregrasp FK and mesh prior to
select its ROI and uses world-ray stride 4; default real ROI settings remain
separately configurable. World-camera initialization reads immutable body
metadata without taking the world lock so dependency loading during an active
trajectory cannot reverse the controller manager's lock order.

The side-corrected nominal `ee_base` mount has optical origin `[0,0,-.100]` m
and aims at `[0,.250,0]` m. Thus EE origin is 100 mm along EE-base +Z from the
camera origin, and the object reference point is 250 mm along EE-base +Y from
the EE origin. The optical frame is X right, Y down, Z forward.
[mount.nominal.json](resource/mount.nominal.json) records this design transform
with `calibration_verified:false`; it is not a measured hand-eye calibration.
The prior 67/74 object ray sweep used the previous mount and does not qualify
this placement. Existing tactile pad mount poses are defined by the established
robot URDF.

The default stream is 640×480 at 30 FPS, matching the manufacturer's
[D405 startup guidance](https://support.realsenseai.com/hc/en-us/community/posts/22773138041107-Realsense-D405-cannot-start).
Depth deprojection uses the SDK's actual intrinsics and depth scale; coordinate
conventions follow the [RealSense projection documentation](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/).

`d405_nonreal_smoke` loads only the nonreal factory path and checks raw depth,
synthetic ROI metadata, AV1 packets and the CameraInfo service. Its arguments are
the plugin library, a harmless existing URDF fixture, and an isolated install
prefix containing this package config plus the existing Raisin network config.
It provides a message-path check; hardware calibration and grasp performance
remain outside this check.

The existing Railab `dev → ffmpeg` GUI window displays the D405 stream.
Connect `railab_raipal`, click **Refresh topics**, then select
`railab_raipal/d405/depth_compressed`. This reuses the existing camera GUI;
no D405-specific viewer is needed.
