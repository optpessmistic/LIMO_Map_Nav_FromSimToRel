# Pinned upstream patches

Both SLAM patches apply in order to official SLAM Toolbox 2.8.5,
commit `ec8f7635dea317b531c419f798f87d90a336f32e`.

`0002-slam-toolbox-inclusive-grid.patch` adds one cell to each
occupancy-grid dimension so the maximum quantized scan endpoint is valid. The
origin, scans, matching and thresholds are unchanged. Its prior validation is
recorded in [the boundary investigation](../docs/slam-grid-boundary.md).

`0003-slam-toolbox-steady-transform-publish.patch` changes exactly one line in
`SlamToolbox::publishTransformLoop`: `rclcpp::Rate` becomes `rclcpp::WallRate`.
In Jazzy, this changes the publication-loop sleep clock from `SYSTEM_TIME` to
`STEADY_TIME`, so a backward system-clock adjustment cannot extend that sleep.
See [the pinned loop](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/src/slam_toolbox_common.cpp#L484)
and [rclcpp clock selection](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/rate.hpp).
It leaves the TF message timestamp, `restamp_tf=false` profile, scan processing,
publish period and every transform tolerance unchanged. It does not make delayed
scan data fresh or remove consumer TF checks. The patch is motivated by measured
WSL system-clock backsteps. The full package has now been built in an isolated
workspace and exercised in run30; this is limited evidence, not full mission
acceptance. The run recorded no navigation TF error 102 or extrapolation log
lines, completed its map criterion, marked two red surrogate targets and verified
return. Visual coverage remained false and the strict retention audit passed only
33/36 entries, leaving the supervisor exit code at 1.
[Clock/build/run30 evidence and limitations](../docs/wsl-clock-tf.md) distinguish
fewer TF receipt gaps from slower worst-case advancement of scan timestamps;
neither zero logged errors nor the successful build proves a general timing fix.

Fetch/apply and then verify without writes:

```bash
bash scripts/fetch_slam_vendor.sh
bash scripts/fetch_slam_vendor.sh --check-only
```

The script accepts only the exact pinned pristine checkout, the exact `0002`
prefix, or both exact patches. It applies missing patches in order, refuses
unknown/out-of-order edits before applying anything, and never resets an existing
checkout. `--check-only` requires both patches and `COLCON_IGNORE` to exist.

Build a **new, separate overlay** from this validated source; preserve the existing
`outputs/slam-boundary-ws` as the historical boundary-only baseline. For example,
use a fresh `outputs/slam-steady-ws` source snapshot and the isolated colcon
procedure in the boundary document, changing its workspace path accordingly.
Do not reuse an existing destination or silently source the new overlay into a
running experiment. Record its source hashes, build results and actually loaded
library paths before evaluating timing. No system ROS installation is replaced.
Retain upstream LICENSE and source notices; `vendor/slam_toolbox/COLCON_IGNORE`
keeps the source out of ordinary project colcon discovery.

The completed local build used `/tmp/limo-slam-steady-20261103`; its relocated
installation is `outputs/slam-steady-ws/install` in the C-drive checkout. Load it
after the F-drive runtime environment and before the current checkout's main
`install/local_setup.bash`. The old boundary-only overlay remains available as
the historical baseline. See the [build receipt](../experiments/run30/validation/slam-steady-build-receipt.json)
and [current GUI startup commands](../docs/visualization.md).

## LIMO Harmonic model

Source: <https://github.com/nevlife/hunter_gazebo> at
`8e1d4aeb459e77d549262ca20cac819d9c7a32ee`.

`0001-limo-harmonic-project-interfaces.patch` changes only the downloaded vendor
checkout. The model files and meshes remain in the ignored `vendor/` directory.
Use `bash scripts/fetch_sim_vendor.sh` to fetch and apply the patch; it is
idempotent and refuses an unexpected repository or commit.

The patch renames the model-only package to `limo_harmonic_description`, avoiding
the name of AgileX's physical chassis driver `limo_base`. It installs the native
Harmonic sensor model from the same upstream checkout, assigns explicit laser
and IMU frame IDs, adds a ROS optical camera frame, and uses Harmonic's camera
`optical_frame_id` for RGB, depth, CameraInfo and point-cloud headers. It also
uses conservative simulation velocity and acceleration limits.

The native DART contact profile uses spherical wheel collisions with the same
wheel centers/radii, exact pi rotations, collision-local axial friction direction,
and lateral slip compliance 0.035 (longitudinal 0), following Gazebo's skid-steer
example. Visual meshes, masses and sensor poses remain inherited. The DiffDrive
separation is derived from the model's collision centers (`track + wheel_width`,
0.175 m). This is a simulation contact approximation, not measured LIMO tire
physics or a change to the physical chassis driver's calibration. The expanded
collision width is 0.265 m and remains inside the padded navigation footprint.
Independent native-pose/IMU checks are required; wheel odometry alone is insufficient.

The X2L has a documented 360-degree nominal emission FOV (the supplied LIMO
manual, PDF page 46, section 5.1). A 360-degree run measured extensive rear
self-returns in this model, so `lidar_fov_deg` defaults to the upstream 240-degree
front sector; 360 remains selectable for occlusion experiments.
This does not certify installed visibility: body occlusion remains modeled, and
the inherited sample count, update rate and range/noise settings are a simulation
baseline rather than a calibrated X2L hardware profile.

The model package retains its upstream BSD-3-Clause license and the AgileX / nevlife
copyright notices. The sensor Xacro comes from the upstream Apache-2.0
`gazebo_harmonic` package; retain the upstream root LICENSE with any redistribution.
Our Git repository records the source pin and modifications rather than vendoring
third-party meshes.
