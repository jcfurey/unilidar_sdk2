# unitree_lidar_ros2

ROS 2 driver for the Unitree LiDAR L2.

Built, tested and smoke run against **Jazzy** (the primary target), **Kilted**
and **Rolling** - the development line that becomes **Lyrical** - with no compiler
warnings on any of them. CI covers all three and Jazzy gates the build.

It also compiles and passes its unit tests on **Humble**, which is not in the CI
matrix: Humble's uncrustify predates the current ament style rules, so its style
check disagrees with sources formatted for Jazzy and newer.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `unilidar/cloud` | `sensor_msgs/PointCloud2` | Fields `x`, `y`, `z`, `intensity`, `ring` (uint16, zero-based scan-line index), `time` (float32 seconds relative to the header stamp), `point_step` 32. |
| `unilidar/imu` | `sensor_msgs/Imu` | Orientation, angular velocity and linear acceleration. |
| `unilidar/laserscan` | `sensor_msgs/LaserScan` | Only when the lidar runs in 2D mode (`work_mode` bit 1). |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Stream rates/ages, cloud span/size, packet gaps, SDK backlog, discarded data and versions. |
| `/tf` | `tf2_msgs/TFMessage` | Optional `<imu_frame>_initial` -> `<imu_frame>` from raw IMU orientation; disabled by default. |
| `/tf_static` | `tf2_msgs/TFMessage` | `<imu_frame>` -> `<cloud_frame>` mounting offset, published once. |

The point cloud layout is byte for byte what earlier releases produced through
`pcl::toROSMsg()`, so consumers configured with these field names and offsets -
most SLAM front ends - need no changes. The driver no longer depends on PCL.

## Build

```bash
cd unitree_lidar_ros2
colcon build
source install/setup.bash
```

The package first looks for an installed `unilidar_sdk2` CMake package, then
falls back to the pre-built SDK next to it in this repository. To use another
source-tree copy directly:

```bash
colcon build --cmake-args -DUNILIDAR_SDK_DIR=/path/to/unitree_lidar_sdk
```

## Run

```bash
ros2 launch unitree_lidar_ros2 launch.py                       # driver + rviz2
ros2 launch unitree_lidar_ros2 launch.py rviz:=false
ros2 launch unitree_lidar_ros2 launch.py config_file:=/path/to/my.yaml
ros2 launch unitree_lidar_ros2 composed_launch.py              # in a component container
```

All settings live in [`config/unilidar_l2.yaml`](config/unilidar_l2.yaml), which
documents each one. `ros2 param describe /unitree_lidar_ros2_node <name>` prints
the same descriptions at run time.

The lidar ships in ethernet mode on `192.168.1.62` and expects this host to be
`192.168.1.2`. For serial, set `transport: serial`. The old numeric
`initialize_type` parameter remains as a deprecated compatibility override.

For a DHCP-managed Ethernet interface, set `local_ip: auto`; the driver derives
the current local IPv4 address from the route to the lidar. `lidar_ip` accepts a
numeric address or a resolvable DNS/mDNS hostname, so a sensor lease can be
addressed through a DHCP reservation or local DNS entry. The SDK does not expose
a command that enables DHCP in the lidar firmware itself.

Cloud, IMU and LaserScan publishers use best-effort sensor-data QoS by default,
with independent profiles and depths. Standard ROS publisher QoS overrides are
enabled as well. A reliable subscriber does not match a best-effort publisher,
so consumers should normally use `SensorDataQoS` or override the relevant driver
publisher to reliable.

### Composition

The driver is also a composable node, `unitree_lidar_ros2::UnitreeLidarNode`.
Loading it into the same container as whatever consumes the cloud lets each scan
pass by intra-process delivery rather than being serialised over loopback
(~160 kB per cloud). See `launch/composed_launch.py`.

### Timestamps and mapping

`timestamp_mode: device` is the mapping default. Cloud, IMU and LaserScan use the
integer seconds/nanoseconds clock carried by their packets, and the driver asks
the lidar to synchronise that clock to the host once at startup. This keeps
cloud headers, IMU samples and every point's relative `time` in one clock domain.

`timestamp_mode: arrival` instead samples the node clock per packet. Cloud and
LaserScan stamps are shifted back by their scan period so their headers still
denote the first measurement, and accumulated point offsets remain relative to
the cloud header. Arrival mode avoids an unsynchronised device clock but includes
transport and scheduling jitter. `use_sim_time` forces this mode.

The old `use_system_timestamp` and `timestamp_source` parameters remain available
through `timestamp_mode: auto`, but are deprecated.

The driver preserves per-point timing; it does not motion-compensate points.
Mapping frontends must deskew with synchronised IMU/odometry. Larger
`cloud_scan_num` values increase the interval that needs deskewing. The accepted
range is 1..1000, but at 300 possible points per line the upper limit can produce
a 300,000-point, 9.6 MB cloud and should be treated as a guardrail rather than an
operating recommendation.

## Notes and known limitations

- **The SDK cannot release the lidar connection.** `closeUDP()` and
  `closeSerial()` in the pre-built `libunilidar_sdk2.a` recurse without bound
  inside `UDPHandler::~UDPHandler()` and crash the process whenever a connection
  is actually open, and `UnitreeLidarReader` has no virtual destructor so the
  reader cannot be deleted either. The driver therefore leaks the reader
  (about 10 kB, once) and holds the port until the process exits. Unloading the
  node from a running component container leaves the port claimed; use a fresh
  container to reload it.
- **`work_mode` is stored in the lidar** and survives power cycles.
  `set_work_mode` therefore defaults to false. A mode that selects a different
  transport from the active connection is rejected unless
  `allow_work_mode_transport_switch` explicitly confirms the persistent switch.
- The IMU quaternion is read as `(x, y, z, w)`, matching the SDK's own example
  output. Releases before 2.0.11 published the transform as `(w, x, y, z)` while
  publishing the message as `(x, y, z, w)`, so the two disagreed. Note the ROS 1
  package in this repository still uses `(w, x, y, z)` for both.
- IMU samples whose quaternion is degenerate - all zeros when the IMU is disabled
  by `work_mode` bit 2 - are dropped rather than published, because tf2 rejects
  them. IMU covariances are configurable; their default all-zero values mean
  unknown. The vendor documentation does not state the quaternion's inertial
  world convention, so validate its axes on hardware before feeding orientation
  into a mapping filter.
- `publish_imu_tf` defaults to false. A robot state estimator should own dynamic
  world/odom transforms. Enable the raw `<imu_frame>_initial` -> `<imu_frame>`
  transform only for explicit compatibility or visualisation use.
- `imu_to_lidar_translation` and `imu_to_lidar_rotation` describe the sensor's
  internal IMU-to-optical geometry. Put the sensor-to-robot mounting transform
  in the robot URDF rather than folding it into these parameters.
- The driver discards the opaque SDK's malformed first point cloud, accumulates
  one-line SDK output itself, assigns zero-based rings, and drops a partial cloud
  when the point-packet sequence has a gap or moves backwards.
