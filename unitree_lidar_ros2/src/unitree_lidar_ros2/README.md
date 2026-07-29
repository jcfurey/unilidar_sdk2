# unitree_lidar_ros2

ROS 2 driver for the Unitree LiDAR L2.

Built, tested and smoke run against **Humble**, **Jazzy** (the primary target),
**Kilted** and **Rolling** - the development line that becomes **Lyrical** - with
no compiler warnings on any of them. CI covers all four; Jazzy is the one that
gates the build.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `unilidar/cloud` | `sensor_msgs/PointCloud2` | Fields `x`, `y`, `z`, `intensity`, `ring` (uint16), `time` (float32, seconds relative to the header stamp), `point_step` 32. |
| `unilidar/imu` | `sensor_msgs/Imu` | Orientation, angular velocity and linear acceleration. |
| `unilidar/laserscan` | `sensor_msgs/LaserScan` | Only when the lidar runs in 2D mode (`work_mode` bit 1). |
| `/tf` | `tf2_msgs/TFMessage` | `<imu_frame>_initial` -> `<imu_frame>` from the IMU orientation. |
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

The package finds the pre-built SDK next to it in this repository. To build
against a copy somewhere else:

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
`192.168.1.2`. For serial, set `initialize_type: 1`.

### Composition

The driver is also a composable node, `unitree_lidar_ros2::UnitreeLidarNode`.
Loading it into the same container as whatever consumes the cloud lets each scan
pass by intra-process delivery rather than being serialised over loopback
(~160 kB per cloud). See `launch/composed_launch.py`.

### Simulated time

With `use_sim_time` enabled, the lidar's own wall-clock stamps are meaningless,
so `timestamp_source` is forced to `ros` and messages are stamped from the node
clock on arrival. Set `timestamp_source` explicitly to silence the warning.

## Notes and known limitations

- **The SDK cannot release the lidar connection.** `closeUDP()` and
  `closeSerial()` in the pre-built `libunilidar_sdk2.a` recurse without bound
  inside `UDPHandler::~UDPHandler()` and crash the process whenever a connection
  is actually open, and `UnitreeLidarReader` has no virtual destructor so the
  reader cannot be deleted either. The driver therefore leaks the reader
  (about 10 kB, once) and holds the port until the process exits. Unloading the
  node from a running component container leaves the port claimed; use a fresh
  container to reload it.
- **`work_mode` is stored in the lidar** and survives power cycles, so launching
  with a `work_mode` that disagrees with `initialize_type` reconfigures the
  device for the *other* transport. The driver warns when it spots this; pass
  `set_work_mode: false` to leave the stored mode alone.
- The IMU quaternion is read as `(x, y, z, w)`, matching the SDK's own example
  output. Releases before 2.0.11 published the transform as `(w, x, y, z)` while
  publishing the message as `(x, y, z, w)`, so the two disagreed. Note the ROS 1
  package in this repository still uses `(w, x, y, z)` for both.
- IMU samples whose quaternion is degenerate - all zeros when the IMU is disabled
  by `work_mode` bit 2 - are dropped rather than published, because tf2 rejects
  them. Without IMU data there is no `<imu_frame>_initial` -> `<imu_frame>`
  transform, so keep the rviz fixed frame on `cloud_frame`.
