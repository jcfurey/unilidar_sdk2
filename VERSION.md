# Version History

## v2.0.0 (2024.10.25)
- First version for Unitree Lidar L2

## v2.0.1 (2024.11.04)
- Update README.md
- Add library file for aarch environment
- Add support for ROS and ROS2

## v2.0.2 (2024.11.05)
- Add support for start and stop lidar rotation

## v2.0.3 (2024.11.14)
- Add udp connect information
- Modify `startLidar()` to `startLidarRotation()`, modify `stopLidar()` to `stopLidarRotation()`

## v2.0.4 (2024.12.10)
- Solve the problem of not able to start up lidar when it's in STANDBY mode

## v2.0.5 (2024.12.13)
- Add function `setLidarIpAddressConfig()` to set lidar ip address
- Add function `setLidarMacAddressConfig()` to set lidar mac address

## v2.0.6 (2024.12.19)
- Add function to get buffer size

## v2.0.7 (2024.12.19)
- Solve the bug to set lidar ip address correctly.

## v2.0.8 (2025.02.25)
- Add function `resetLidar()` to restart lidar hardware
- Modify namespace from `unitree_lidar_sdk` to `unilidar_sdk2`
- Modify library file name from `libunitree_lidar_sdk.a` to `libunilidar_sdk2.a`

## v2.0.9 (2025.03.04)
- Add function `sendUserCtrlCmd()` to send `LidarUserCtrlCmd` to lidar

## v2.0.10 (2025.03.10)
- Solve the bug of not displaying 2D LaserScan msg, 
- the default frame of 2D scan is changed to `unilidar_laserscan`

## v2.0.11
Prepares the SDK and the ROS 2 driver for ROS 2 Jazzy and Lyrical.

### SDK
- `cmake_minimum_required` raised from 3.0 to 3.16. CMake 4.0 removed
  compatibility with CMake < 3.5, so the project no longer configured at all with
  the CMake shipped by recent distributions.
- Export an imported target `unilidar::sdk2` carrying the include path and the
  pthread dependency, map `aarch64`/`arm64` onto the shipped library directories,
  and stop overwriting `CMAKE_CXX_FLAGS` to set the C++ standard.
- `transformUnitreeCloudToPCL()` in `unitree_lidar_sdk_pcl.h` is now `inline`:
  including that header from more than one translation unit failed to link.
  It also sizes the output cloud once instead of growing it point by point, and
  fills in PCL's homogeneous coordinate, which was left uninitialised.
- The blanket `using namespace unilidar_sdk2;` in `unitree_lidar_sdk_pcl.h` can be
  suppressed with `UNILIDAR_SDK_PCL_NO_GLOBAL_NAMESPACE`.
- `parseFromPacketToPointCloud()` and `parseFromPacketPointCloud2D()` clamp the
  packet's `point_num` to the capacity of the arrays it indexes, so a corrupted
  or truncated packet can no longer read past the end of the packet. Both also
  reserve the right number of points up front.
- Added compile-time asserts on every protocol struct size, and corrected the
  sizes quoted in the comments: several had drifted (`LidarImuData` is 56 bytes,
  not 132; `LidarPointDataPacket` is 1044, not 1036).
- `example.h` no longer reads past the end of clouds with fewer than 10 points,
  and prints the nanosecond field zero padded so that stamps such as
  `1.004411172` are not shown as `1.4411172`.
- `unitree_lidar_sdk_VERSION` had been left at 2.0.9.

### ROS 2 driver
- Built with `ament_cmake_auto`, and available as a composable node
  (`unitree_lidar_ros2::UnitreeLidarNode`) in addition to the standalone
  `unitree_lidar_ros2_node` executable.
- The lidar is drained by a dedicated thread instead of a 1 ms wall timer on the
  executor. The SDK sets a one second receive timeout on its socket, so a read
  could previously stall the whole node.
- The IMU orientation is read as `(x, y, z, w)` for both the message and the
  transform. The transform used to read the same array as `(w, x, y, z)`, so the
  two disagreed. Degenerate quaternions - all zeros when the IMU is disabled -
  are dropped instead of being published for tf2 to reject.
- The `<imu_frame>` -> `<cloud_frame>` offset is published once on `/tf_static`
  rather than being re-sent with every IMU sample, and transforms are stamped
  with the sample time instead of `now()`.
- New `timestamp_source` parameter, forced to `ros` under `use_sim_time` because
  the lidar's wall-clock stamps are meaningless against simulated time. Stamps
  are built as `RCL_ROS_TIME` and computed without losing the low digits of a
  Unix timestamp to a double.
- A failed `initializeSerial()`/`initializeUDP()` is now reported and exits
  non-zero; the return value used to be ignored, and an unusable
  `initialize_type` called `exit(0)`.
- `sensor_msgs/LaserScan` is published in 2D mode, matching the ROS 1 package.
  Invalid returns are marked infinite rather than 0.0 m, `scan_time` is filled
  in, and the calibration angle bias is applied.
- The point cloud is assembled directly into the message, so the package no
  longer depends on PCL and no longer copies each cloud through a
  `pcl::PointCloud`. The binary layout is unchanged.
- `lidar_ip` and `local_ip` had their default values swapped.
- `lidar_ip` now accepts DNS/mDNS hostnames, and `local_ip: auto` derives the
  host address selected by the route. This supports host and sensor addresses
  managed by DHCP without hard-coding the current host lease in the driver.
- New parameters: `set_work_mode`, `laserscan_topic`, `laserscan_frame`,
  `publish_imu_tf`, `publish_static_tf`, `imu_to_lidar_translation`,
  `start_rotation_on_startup`, `stop_rotation_on_shutdown`, `qos_profile`,
  `qos_depth`, `idle_sleep_us`, `watchdog_timeout`. All parameters carry
  descriptions and ranges, and `range_max` now defaults to 100 m as elsewhere.
- Warns when `work_mode` selects a different transport than `initialize_type`,
  which silently reconfigured the lidar for the next power cycle.
- A watchdog warns when no packet arrives, with the detail needed to work out why.
- `launch.py` uses the package share directory instead of shelling out to
  `ros2 pkg prefix`, and takes `config_file`, `rviz`, `rviz_config`, `namespace`,
  `use_sim_time` and `log_level` arguments.
- Include tf2_ros's `.hpp` headers where they exist. The `.h` shims are deprecated
  from Rolling onwards and warn on every build; Humble only ships the `.h` form,
  so the choice is made with `__has_include`.
- Pass `USE_SCOPED_HEADER_INSTALL_DIR` to `ament_auto_package()`, which installs
  headers to `include/unitree_lidar_ros2/`. That becomes the default in Kilted, so
  asking for it keeps the layout the same on every distribution.
- Stopped using `ament_target_dependencies()`, which no longer exists on Rolling;
  linking the component target propagates its dependencies instead.
- Test code uses `SingleThreadedExecutor::spin_some()` rather than
  `rclcpp::spin_some(node)`, which is deprecated from Rolling on.
- Builds warning free on Humble, Jazzy, Kilted and Rolling.

### Tests and linting
- `ament_lint_auto` now runs cpplint, uncrustify, flake8 and lint_cmake over the
  package. ament_copyright and ament_pep257 are left out because the copyright
  headers are the vendor's, and ament_xmllint because it validates package.xml
  against a schema it fetches over the network.
- 35 test cases across four executables:
  - `test_conversions` covers the timestamp arithmetic (including that splitting a
    Unix timestamp before scaling is what keeps the seconds exact), the IMU
    quaternion normalisation and its rejection of the all zero quaternion the
    lidar sends with its IMU disabled, the 2D range window, and the point count
    clamp.
  - `test_point_cloud_layout` pins the published cloud to the layout
    `pcl::toROSMsg()` used to emit.
  - `test_sdk_reference_parsers` covers the reference parsers in
    `unitree_lidar_utilities.h`, including that a packet claiming more points than
    the array holds is clamped rather than read out of bounds.
  - `test_driver_end_to_end` runs the real node against synthesised UDP packets
    and checks what it publishes: the IMU orientation and its transform agreeing
    on `(x, y, z, w)`, the transform carrying the sample's own stamp, the cloud's
    fields and geometry, the latched mounting transform, degenerate quaternions
    being dropped, and a malformed point count neither crashing the driver nor
    stopping it.
- The pure conversions moved into `conversions.hpp`, which has no ROS dependency
  so it can be tested on its own.
- Corrected the comment on `FrameTail::crc32`: the checksum covers the payload
  only, not "head and data". Established by testing the pre-built parser, which
  rejects a checksum computed over header + payload.

### Known SDK limitations
- `closeUDP()` and `closeSerial()` recurse without bound inside
  `UDPHandler::~UDPHandler()` and crash the process whenever a connection is
  actually open. `UnitreeLidarReader` also has no virtual destructor, so the
  reader cannot be deleted through the base pointer. The ROS 2 driver therefore
  holds the connection until the process exits.

## v2.0.12 (2026.08.19)

### SDK and packaging
- The SDK installs its headers, both architecture archives and a relocatable
  `unilidar_sdk2` CMake config exporting `unilidar::sdk2`.
- Examples now default off and remain in the build tree instead of writing
  binaries into the source checkout.
- The reference 2D and 3D parsers treat a firmware range window of `0 .. 0` as
  unspecified rather than dropping every return.
- CI compiles and install-tests the SDK on native x86_64 and aarch64 runners.

### ROS 2 driver
- `transport: ethernet|serial` replaces the numeric `initialize_type`; the old
  parameter remains as a deprecated compatibility override.
- Persistent work-mode writes default off, the bitfield is constrained to its
  five documented bits, and transport changes require an explicit confirmation.
- Point cloud, IMU and LaserScan publishers have independent sensor-data QoS and
  support standard ROS publisher QoS overrides.
- `/diagnostics` reports stream rates, packet/data age, connection details and
  hardware, firmware and SDK versions.
- Empty or malformed 2D scan geometry is dropped instead of publishing an
  invalid `LaserScan`, and the internal static transform now accepts a complete
  quaternion as well as translation.
- All ROS setup occurs before the connection is opened, reducing the chance that
  a failed component construction strands the vendor SDK's unreleasable port.

### Tests
- 41 cases cover pure conversions, protocol/reference parsers, UDP publication,
  diagnostics and a real 4 Mbaud SDK serial open against a pseudo-terminal.
