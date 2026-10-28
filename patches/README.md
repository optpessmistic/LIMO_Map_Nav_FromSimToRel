# LIMO Harmonic vendor patch

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

The model package retains its upstream BSD-3-Clause license and the AgileX / nevlife
copyright notices. The sensor Xacro comes from the upstream Apache-2.0
`gazebo_harmonic` package; retain the upstream root LICENSE with any redistribution.
Our Git repository records the source pin and modifications rather than vendoring
third-party meshes.
