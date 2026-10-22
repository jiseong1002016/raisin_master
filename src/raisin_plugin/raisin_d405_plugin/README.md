# raisin_d405_plugin

Real mode captures D405 depth and RGB with librealsense and aligns the color
image to the depth optical frame (`rs2::align(RS2_STREAM_DEPTH)`). Nonreal mode
uses an offscreen **rayrai** RGB and linear-depth render of the same RaiSim
world, camera pose and pinhole intrinsics. Neither backend predicts a grasp.
Both publish `raisin_d405_grasp_input_v2` for the grasp backend's shared
RGBD → robot self filter → AnyGrasp → camera/world/policy transform.

The capture thread owns its GL context in nonreal mode and never opens USB.
Both rayrai passes hold the world mutex, so RGB and depth share one robot/world
state. GPU readback, publication and encoding happen after releasing the mutex.
Real capture errors stop acquisition; there is no synthetic fallback.

`simulation_source: rayrai` is the default. The older `plane` and CPU collision
`world` sources remain explicit depth-only transport/test fixtures. Their v1
packets cannot enter AnyGrasp initialization. `simulation_camera_to_ee_json`
lets the launcher supply the exact transform composed from the selected URDF,
locked EE base angle and latest D405 mount; otherwise the plugin composes its
installed nominal mount with the canonical EE mount YAML.

Default prefix `railab_raipal/d405` exposes:

- `/depth`: `sensor_msgs::Image`, `32FC1`, optical-axis depth in meters.
- `/rgb`: `sensor_msgs::Image`, `rgb8`, in the same depth optical frame.
- `/depth/camera_info` and `/depth/get_camera_info`: native camera intrinsics.
- `/depth_compressed`: existing AV1 GUI transport, encoded when subscribed.
- `/grasp_input`: JSON in `std_msgs::String`, schema `raisin_d405_grasp_input_v2`.
- `/status`: acquisition errors.

The v2 packet has `acquisition_source` (`d405` or `rayrai`), `is_simulation`,
`sequence`, `stamp_monotonic_ns`, `frame_id`, ROI/depth statistics, actual depth
intrinsics, and at most 12,000 paired `points_camera_m` / `colors_rgb_u8` rows.
XYZ is deprojected at each sampled depth pixel; its RGB comes from that same
aligned pixel. `stamp_monotonic_ns` is the host receive time for real frames
and the time immediately before the locked render for rayrai. It is not a
hardware exposure timestamp. Use a 4 MiB backend message bound for RGBD input.
The ROI must include the selected object's visible surface.

The current nominal mount places the optical origin at `[0,0,-.100]` m in
`ee_base` and aims at `[0,.250,0]` m. Optical axes are X right, Y down, Z forward.
`resource/mount.nominal.json` records its source and still declares
`calibration_verified:false`; a configuration match does not certify a measured
hand-eye calibration.

Build with `bash scripts/build.sh` (one compiler process). `D405_BUILD_DIR` and
`D405_INSTALL_DIR` can select isolated build/install directories. The build
requires the Raisin SDK, librealsense, OpenCV, yaml-cpp and rayrai.

`d405_nonreal_smoke` explicitly selects the plane fixture and checks existing
depth, JSON, AV1 and CameraInfo transports. `d405_rayrai_rgbd_check` renders a
known red box, checks its independently predicted 0.200 m front-face depth and
RGB channel order, without camera USB or robot I/O. These checks cover transport
and rendering, not physical calibration, AnyGrasp inference or grasp success.
