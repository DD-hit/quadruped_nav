# Upstream and local integration patches

real-lsy/That-nav, commit 38b12d8d83d10ad1fe1e6b354a34fbf717ae42f4,
go2w_nav_ws/src/FAST_LIO_ROS2. Only build/runtime files extracted; large docs omitted.
Original source hashes are in third_party/fastlio_source_manifest.json, and originals
in third_party/reference/that_nav. Retain all original licenses and file notices.

Changes: modern CMake with explicit dependencies; remove unused Python embedding
and pcl_ros build dependency; explicit OpenMP header for preprocess compilation;
output_directory parameter routes logs/PCD into per-run data; publish current pose
covariance in the current message instead of one frame late. IMU integration,
scan compensation, matching and state update equations remain unchanged.

The body frame remains the IMU; TF is isolated by the project launcher. Simulation
clock rollback requires a fresh estimator process, managed by lio_runtime.py.

2026-11-05 output-only changes: export estimated gravity in camera_init at the odometry stamp; export IMU-origin child-frame twist; correct position/attitude covariance order and rotate right-local attitude errors into ROS fixed reference axes. Twist covariance is an approximate state-Jacobian propagation with configured gyro noise; calibration and sample/posterior correlations are not modeled. Filter equations, integration, matching and deskewing remain unchanged. See docs/base_odometry.md.
