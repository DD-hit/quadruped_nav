# livox_ros_driver2 — message-only extraction

Source: real-lsy/That-nav commit 38b12d8d83d10ad1fe1e6b354a34fbf717ae42f4,
`go2w_nav_ws/src/livox_ros_driver2/msg/{CustomPoint,CustomMsg}.msg`.

Both message definitions are byte-identical to that commit, retaining the original
ROS package/type names. CMakeLists.txt and package.xml are project replacements
which generate interfaces only. No driver code, SDK library or hardware launch is
included. The upstream package declares MIT; its originals and repository-wide
Apache-2.0 LICENSE are preserved under third_party/reference/that_nav (the latter
is also copied here as UPSTREAM_LICENSE).

Do not source this package and a separate copy of the hardware driver into the
same build environment. A future hardware workspace should build the full driver
instead of this extraction, with the message definitions checked for compatibility.
The generated Python extension targets system ROS Python 3.10, not Isaac Python.
