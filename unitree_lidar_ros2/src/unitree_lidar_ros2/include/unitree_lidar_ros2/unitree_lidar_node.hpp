/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef UNITREE_LIDAR_ROS2__UNITREE_LIDAR_NODE_HPP_
#define UNITREE_LIDAR_ROS2__UNITREE_LIDAR_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
// tf2_ros renamed its headers to .hpp. The .h shims warn on every build from
// Rolling (Lyrical) onwards and are slated for removal, while Humble only ships
// the .h form, so pick whichever the installed tf2_ros provides.
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

// SDK. Included through the plain C++ interface only: this driver deliberately
// does not use unitree_lidar_sdk_pcl.h, so the package does not depend on PCL.
#include "unitree_lidar_sdk.h"  // NOLINT(build/include_subdir)

namespace unitree_lidar_ros2
{

/// How the connection to the lidar is opened. Matches the historical numbering
/// of the `initialize_type` parameter.
enum class ConnectionType : int
{
  Serial = 1,
  Udp = 2,
};

/// Which clock the published stamps come from.
enum class TimestampSource
{
  /// The stamp carried by the SDK payload: the lidar's own clock, or the host
  /// clock at packet assembly time when `use_system_timestamp` is set.
  Sensor,
  /// The node clock, sampled when the packet is handed to this node. This is the
  /// only meaningful choice when `use_sim_time` is enabled.
  Ros,
};

/**
 * @brief Deleter for the reader handed out by createUnitreeLidarReader().
 *
 * It deliberately releases nothing, because neither way of releasing the reader
 * is currently safe:
 *
 *  - `delete reader` is undefined behaviour. unilidar_sdk2::UnitreeLidarReader
 *    declares no virtual destructor, so the concrete implementation would not be
 *    destroyed properly. Adding one to the header is not an option either: that
 *    inserts entries into the vtable and breaks the ABI of the pre-built
 *    libunilidar_sdk2.a.
 *
 *  - `closeUDP()` and `closeSerial()` crash. In the shipped library (VERSION.md
 *    2.0.10, getVersionOfSDK() reports 2.0.9) closing a connection that is
 *    actually open recurses without bound inside UDPHandler::~UDPHandler() and
 *    takes the process down with a stack overflow. Calling close on a reader
 *    that was never connected is harmless, which is exactly the case where it
 *    achieves nothing.
 *
 * So the reader and its socket or serial port live until the process exits. That
 * matches the process lifetime for the standalone node. Inside a component
 * container it means unloading the driver leaves the port claimed, so reloading
 * it needs a fresh container until the SDK is fixed.
 *
 * The type is kept so that ownership is still expressed through a unique_ptr and
 * there is a single place to undo this once the SDK can release cleanly.
 */
struct LidarReaderRelease
{
  void operator()(unilidar_sdk2::UnitreeLidarReader * reader) const noexcept;
};

using LidarReaderHandle =
  std::unique_ptr<unilidar_sdk2::UnitreeLidarReader, LidarReaderRelease>;

/**
 * @brief Publishes point cloud, IMU and 2D scan data from a Unitree L2 lidar.
 *
 * The lidar is drained by a dedicated thread rather than by a timer on the
 * executor, so a slow or blocking read cannot stall the rest of the node (or the
 * rest of the process when the node runs inside a component container). Every
 * call into the SDK happens either on that thread, or on the thread that runs
 * the constructor/destructor while the poll thread is known not to be running.
 */
class UnitreeLidarNode : public rclcpp::Node
{
public:
  explicit UnitreeLidarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~UnitreeLidarNode() override;

  UnitreeLidarNode(const UnitreeLidarNode &) = delete;
  UnitreeLidarNode & operator=(const UnitreeLidarNode &) = delete;
  UnitreeLidarNode(UnitreeLidarNode &&) = delete;
  UnitreeLidarNode & operator=(UnitreeLidarNode &&) = delete;

private:
  /// Values read from the parameter server, all of them at construction time.
  struct Parameters
  {
    ConnectionType connection{ConnectionType::Udp};
    int work_mode{0};
    bool set_work_mode{true};

    std::string serial_port{"/dev/ttyACM0"};
    int baudrate{4000000};

    int lidar_port{6101};
    std::string lidar_ip{"192.168.1.62"};
    int local_port{6201};
    std::string local_ip{"192.168.1.2"};

    int cloud_scan_num{18};
    bool use_system_timestamp{true};
    std::string timestamp_source{"sensor"};
    double range_min{0.0};
    double range_max{100.0};

    std::string cloud_frame{"unilidar_lidar"};
    std::string cloud_topic{"unilidar/cloud"};
    std::string imu_frame{"unilidar_imu"};
    std::string imu_topic{"unilidar/imu"};
    std::string laserscan_frame{"unilidar_laserscan"};
    std::string laserscan_topic{"unilidar/laserscan"};

    bool publish_imu_tf{true};
    bool publish_static_tf{true};
    std::vector<double> imu_to_lidar_translation{0.007698, 0.014655, -0.00667};

    bool start_rotation_on_startup{true};
    bool stop_rotation_on_shutdown{false};

    std::string qos_profile{"default"};
    int qos_depth{10};

    int idle_sleep_us{100};
    double watchdog_timeout{3.0};
  };

  void declareParameters();
  void openLidar();
  void createInterfaces();
  void publishStaticTransform();

  void pollLoop();
  void stopPolling() noexcept;

  void handleImuPacket();
  void handlePointCloudPacket();
  void handleLaserScanPacket();
  void reportVersionsOnce();
  void checkLiveness();

  /// Resolves the stamp to publish for a payload carrying @p sensor_stamp
  /// seconds since the epoch, honouring `timestamp_source` and simulated time.
  rclcpp::Time resolveStamp(double sensor_stamp);

  Parameters params_;
  TimestampSource timestamp_source_{TimestampSource::Sensor};
  bool use_sim_time_{false};

  LidarReaderHandle lidar_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  /// Field descriptors of the published cloud, built once.
  std::vector<sensor_msgs::msg::PointField> cloud_fields_;

  /// Scratch buffer reused by the poll thread so that every scan does not
  /// reallocate its point vector. Only ever touched by pollLoop(). Braces
  /// matter: PointCloudUnitree has no constructor, so its scalars would
  /// otherwise start out uninitialised.
  unilidar_sdk2::PointCloudUnitree cloud_scratch_{};

  rclcpp::Context::SharedPtr context_;
  std::thread poll_thread_;
  std::atomic<bool> running_{false};

  /// Steady clock reading, in nanoseconds, of the last packet parsed. Written by
  /// the poll thread, read by the watchdog running on the executor.
  std::atomic<int64_t> last_packet_time_ns_{0};
  std::atomic<uint64_t> cloud_count_{0};
  std::atomic<uint64_t> imu_count_{0};
  std::atomic<uint64_t> laserscan_count_{0};

  /// Snapshot of the counters above at the previous watchdog tick, used to report
  /// message rates. Only touched by the watchdog callback.
  int64_t last_rate_report_ns_{0};
  uint64_t reported_cloud_count_{0};
  uint64_t reported_imu_count_{0};
  uint64_t reported_laserscan_count_{0};

  /// Throttled logging must not depend on a ROS clock that may be paused or not
  /// yet publishing under simulated time.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  bool versions_reported_{false};
};

}  // namespace unitree_lidar_ros2

#endif  // UNITREE_LIDAR_ROS2__UNITREE_LIDAR_NODE_HPP_
