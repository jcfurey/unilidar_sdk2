/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_ros2/unitree_lidar_node.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/qos_overriding_options.hpp>

#include "unitree_lidar_ros2/conversions.hpp"
#include "unitree_lidar_ros2/point_cloud_layout.hpp"

namespace unitree_lidar_ros2
{
namespace
{

/// Warn when the lidar clock and the ROS clock disagree by more than this.
constexpr double kClockOffsetWarnSeconds = 1.0;

/// Throttling interval for the repeating diagnostics, in milliseconds.
constexpr int64_t kThrottleMs = 5000;

/// Bit 3 of the lidar work mode selects serial instead of ethernet.
constexpr uint32_t kWorkModeSerialBit = 1u << 3;

std::string resolveIpv4Address(const std::string & host, int port)
{
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  addrinfo * raw_addresses = nullptr;
  const std::string service = std::to_string(port);
  const int result = getaddrinfo(host.c_str(), service.c_str(), &hints, &raw_addresses);
  if (result != 0) {
    throw std::runtime_error(
      "Failed to resolve IPv4 address '" + host + "': " + gai_strerror(result));
  }
  const std::unique_ptr<addrinfo, decltype(& freeaddrinfo)> addresses(
    raw_addresses, freeaddrinfo);
  if (!addresses) {
    throw std::runtime_error("No IPv4 address was returned for '" + host + "'");
  }

  const auto * address = reinterpret_cast<const sockaddr_in *>(addresses->ai_addr);
  char numeric_address[INET_ADDRSTRLEN]{};
  if (inet_ntop(AF_INET, &address->sin_addr, numeric_address, sizeof(numeric_address)) == nullptr) {
    throw std::runtime_error(
      "Failed to format resolved IPv4 address for '" + host + "': " + std::strerror(errno));
  }
  return numeric_address;
}

std::string selectLocalIpv4Address(const std::string & remote_ip, int remote_port)
{
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd < 0) {
    throw std::runtime_error(
      "Failed to create a UDP socket while selecting the local address: " +
      std::string(std::strerror(errno)));
  }

  sockaddr_in remote_address{};
  remote_address.sin_family = AF_INET;
  remote_address.sin_port = htons(static_cast<uint16_t>(remote_port));
  if (inet_pton(AF_INET, remote_ip.c_str(), &remote_address.sin_addr) != 1) {
    close(socket_fd);
    throw std::runtime_error("Resolved lidar address is not IPv4: " + remote_ip);
  }

  if (connect(
      socket_fd, reinterpret_cast<const sockaddr *>(&remote_address),
      sizeof(remote_address)) != 0)
  {
    const std::string error = std::strerror(errno);
    close(socket_fd);
    throw std::runtime_error(
      "Failed to select a route to lidar at " + remote_ip + ":" +
      std::to_string(remote_port) + ": " + error);
  }

  sockaddr_in local_address{};
  socklen_t local_address_size = sizeof(local_address);
  if (getsockname(
      socket_fd, reinterpret_cast<sockaddr *>(&local_address), &local_address_size) != 0)
  {
    const std::string error = std::strerror(errno);
    close(socket_fd);
    throw std::runtime_error("Failed to read the selected local address: " + error);
  }
  close(socket_fd);

  char numeric_address[INET_ADDRSTRLEN]{};
  if (inet_ntop(
      AF_INET, &local_address.sin_addr, numeric_address, sizeof(numeric_address)) == nullptr)
  {
    throw std::runtime_error(
      "Failed to format the selected local IPv4 address: " + std::string(std::strerror(errno)));
  }
  return numeric_address;
}

rcl_interfaces::msg::ParameterDescriptor describe(
  const std::string & description, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeRange(
  const std::string & description, double from, double to, bool read_only = false)
{
  auto descriptor = describe(description, read_only);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeIntRange(
  const std::string & description, int64_t from, int64_t to, bool read_only = false)
{
  auto descriptor = describe(description, read_only);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS makeQos(const std::string & profile, int depth)
{
  const auto history = rclcpp::KeepLast(static_cast<size_t>(std::max(1, depth)));
  if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS(history);
  }
  if (profile == "default") {
    // Reliable and volatile, which is what this driver has always published.
    return rclcpp::QoS(history);
  }
  throw std::invalid_argument(
    "QoS profile must be 'default' or 'sensor_data', got '" + profile + "'");
}

ConnectionType parseTransport(const std::string & name)
{
  if (name == "ethernet" || name == "udp") {
    return ConnectionType::Udp;
  }
  if (name == "serial") {
    return ConnectionType::Serial;
  }
  throw std::invalid_argument(
    "transport must be 'ethernet' or 'serial', got '" + name + "'");
}

diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string formatRate(double rate)
{
  if (!std::isfinite(rate)) {
    return "0.0";
  }
  char text[32]{};
  std::snprintf(text, sizeof(text), "%.1f", rate);
  return text;
}

std::string formatAge(int64_t now_ns, int64_t then_ns)
{
  if (then_ns <= 0) {
    return "never";
  }
  char text[32]{};
  std::snprintf(
    text, sizeof(text), "%.3f",
    std::max(0.0, static_cast<double>(now_ns - then_ns) * 1e-9));
  return text;
}

TimestampSource parseTimestampSource(const std::string & name)
{
  if (name == "sensor" || name == "lidar") {
    return TimestampSource::Sensor;
  }
  if (name == "ros" || name == "now") {
    return TimestampSource::Ros;
  }
  throw std::invalid_argument(
    "timestamp_source must be 'sensor' or 'ros', got '" + name + "'");
}

std::vector<sensor_msgs::msg::PointField> buildCloudFields()
{
  using PointField = sensor_msgs::msg::PointField;

  const auto field = [](const char * name, size_t offset, uint8_t datatype) {
      PointField f;
      f.name = name;
      f.offset = static_cast<uint32_t>(offset);
      f.datatype = datatype;
      f.count = 1;
      return f;
    };

  return {
    field("x", offsetof(PackedPoint, x), PointField::FLOAT32),
    field("y", offsetof(PackedPoint, y), PointField::FLOAT32),
    field("z", offsetof(PackedPoint, z), PointField::FLOAT32),
    field("intensity", offsetof(PackedPoint, intensity), PointField::FLOAT32),
    field("ring", offsetof(PackedPoint, ring), PointField::UINT16),
    field("time", offsetof(PackedPoint, time), PointField::FLOAT32),
  };
}

}  // namespace

void LidarReaderRelease::operator()(unilidar_sdk2::UnitreeLidarReader * reader) const noexcept
{
  // Intentionally empty: neither `delete reader` nor closeUDP()/closeSerial() can
  // be called safely against the pre-built library. See the documentation on
  // LidarReaderRelease for the details of both problems.
  (void)reader;
}

UnitreeLidarNode::UnitreeLidarNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("unitree_lidar_ros2_node", options)
{
  context_ = this->get_node_base_interface()->get_context();
  use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

  declareParameters();

  timestamp_source_ = parseTimestampSource(params_.timestamp_source);
  if (use_sim_time_ && timestamp_source_ == TimestampSource::Sensor) {
    RCLCPP_WARN(
      get_logger(),
      "use_sim_time is enabled but timestamp_source is 'sensor'. The lidar stamps come from a "
      "wall clock and are meaningless against simulated time, so 'ros' is used instead. Set "
      "timestamp_source explicitly to silence this warning.");
    timestamp_source_ = TimestampSource::Ros;
  }

  createInterfaces();
  if (params_.publish_static_tf) {
    publishStaticTransform();
  }

  if (params_.watchdog_timeout > 0.0) {
    const auto period = std::chrono::duration<double>(std::min(1.0, params_.watchdog_timeout));
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {checkLiveness();});
  }
  if (params_.diagnostics_period > 0.0) {
    diagnostics_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.diagnostics_period)),
      [this]() {publishDiagnostics();});
  }

  // Open the hardware only after every ROS interface and timer has been created.
  // The pre-built SDK cannot release an open connection, so a constructor failure
  // after this point would otherwise leave the component container holding the port.
  openLidar();

  RCLCPP_INFO(
    get_logger(),
    "Publishing cloud on '%s' (frame '%s'), imu on '%s' (frame '%s'), 2d scan on '%s' "
    "(frame '%s'); timestamp_source=%s, qos cloud=%s/%d imu=%s/%d scan=%s/%d",
    params_.cloud_topic.c_str(), params_.cloud_frame.c_str(),
    params_.imu_topic.c_str(), params_.imu_frame.c_str(),
    params_.laserscan_topic.c_str(), params_.laserscan_frame.c_str(),
    timestamp_source_ == TimestampSource::Sensor ? "sensor" : "ros",
    params_.cloud_qos_profile.c_str(), params_.cloud_qos_depth,
    params_.imu_qos_profile.c_str(), params_.imu_qos_depth,
    params_.laserscan_qos_profile.c_str(), params_.laserscan_qos_depth);

  // Started last: nothing below may throw, or the thread would be left running
  // with a half constructed node (and a joinable std::thread destructor aborts).
  const int64_t started_ns = steadyNowNs();
  last_packet_time_ns_.store(started_ns, std::memory_order_relaxed);
  last_data_time_ns_.store(started_ns, std::memory_order_relaxed);
  running_.store(true, std::memory_order_relaxed);
  poll_thread_ = std::thread(&UnitreeLidarNode::pollLoop, this);
}

UnitreeLidarNode::~UnitreeLidarNode()
{
  stopPolling();

  // Safe to talk to the SDK again now that the poll thread has been joined.
  if (lidar_ && params_.stop_rotation_on_shutdown) {
    try {
      lidar_->stopLidarRotation();
    } catch (...) {
      // Ignore: we are going away regardless.
    }
  }

  // The connection itself outlives the node; see LidarReaderRelease.
}

void UnitreeLidarNode::declareParameters()
{
  const std::string transport = declare_parameter<std::string>(
    "transport", "ethernet",
    describe("Connection transport: 'ethernet' or 'serial'.", true));
  const int initialize_type = declare_parameter<int>(
    "initialize_type", 0,
    describeIntRange(
      "Deprecated numeric transport override: 1 = serial, 2 = ethernet, 0 uses `transport`.",
      0, 2, true));
  params_.connection = parseTransport(transport);
  if (initialize_type != 0) {
    params_.connection = static_cast<ConnectionType>(initialize_type);
    RCLCPP_WARN(
      get_logger(), "Parameter 'initialize_type' is deprecated; use transport:='%s' instead.",
      params_.connection == ConnectionType::Serial ? "serial" : "ethernet");
  }

  params_.work_mode = declare_parameter<int>(
    "work_mode", 0,
    describeIntRange(
      "Work mode bitfield written to the lidar. Bit 0 wide FOV, bit 1 2D mode, bit 2 disable "
      "IMU, bit 3 serial instead of ethernet, bit 4 wait for a start command after power on.",
      0, 31, true));
  params_.set_work_mode = declare_parameter<bool>(
    "set_work_mode", false,
    describe(
      "Write `work_mode` to the lidar on start up. The lidar stores it across power cycles, so "
      "set this to false to leave the mode currently programmed into the device alone.",
      true));
  params_.allow_work_mode_transport_switch = declare_parameter<bool>(
    "allow_work_mode_transport_switch", false,
    describe(
      "Permit work_mode to select a different transport from the active connection. This "
      "intentionally moves the lidar to that transport after its next restart.",
      true));

  params_.serial_port = declare_parameter<std::string>(
    "serial_port", "/dev/ttyACM0",
      describe("Serial device, used when transport is 'serial'.", true));
  params_.baudrate = declare_parameter<int>(
    "baudrate", 4000000, describeIntRange("Serial baud rate.", 9600, 12000000, true));

  params_.lidar_port = declare_parameter<int>(
    "lidar_port", 6101, describeIntRange("UDP port the lidar sends from.", 1, 65535, true));
  params_.lidar_ip = declare_parameter<std::string>(
    "lidar_ip", "192.168.1.62",
    describe("IPv4 address or resolvable hostname of the lidar.", true));
  params_.local_port = declare_parameter<int>(
    "local_port", 6201, describeIntRange("UDP port this host receives on.", 1, 65535, true));
  params_.local_ip = declare_parameter<std::string>(
    "local_ip", "192.168.1.2",
    describe(
      "IPv4 address of this host's interface, or 'auto' to select it from the route to lidar_ip. "
      "Use 'auto' when the host address comes from DHCP.",
      true));

  params_.cloud_scan_num = declare_parameter<int>(
    "cloud_scan_num", 18,
    describeIntRange("Number of scan lines accumulated into one published cloud.", 1, 1000, true));
  params_.use_system_timestamp = declare_parameter<bool>(
    "use_system_timestamp", true,
    describe(
      "Have the SDK stamp clouds with this host's clock instead of the lidar's own clock.", true));
  params_.timestamp_source = declare_parameter<std::string>(
    "timestamp_source", "sensor",
    describe(
      "'sensor' publishes the stamp carried by the lidar data, 'ros' stamps messages from the node "
      "clock when they are received. Forced to 'ros' when use_sim_time is enabled.",
      true));
  params_.range_min = declare_parameter<double>(
    "range_min", 0.0,
      describeRange("Discard returns closer than this, in metres.", 0.0, 1000.0, true));
  params_.range_max = declare_parameter<double>(
    "range_max", 100.0,
      describeRange("Discard returns further than this, in metres.", 0.0, 1000.0, true));

  params_.cloud_frame = declare_parameter<std::string>(
    "cloud_frame", "unilidar_lidar", describe("frame_id of the published point cloud.", true));
  params_.cloud_topic = declare_parameter<std::string>(
    "cloud_topic", "unilidar/cloud", describe("Point cloud topic.", true));
  params_.imu_frame = declare_parameter<std::string>(
    "imu_frame", "unilidar_imu", describe("frame_id of the published IMU data.", true));
  params_.imu_topic = declare_parameter<std::string>(
    "imu_topic", "unilidar/imu", describe("IMU topic.", true));
  params_.laserscan_frame = declare_parameter<std::string>(
    "laserscan_frame", "unilidar_laserscan",
    describe("frame_id of the 2D scan published in 2D work mode.", true));
  params_.laserscan_topic = declare_parameter<std::string>(
    "laserscan_topic", "unilidar/laserscan", describe("2D LaserScan topic.", true));

  params_.publish_imu_tf = declare_parameter<bool>(
    "publish_imu_tf", true,
    describe("Broadcast <imu_frame>_initial -> <imu_frame> from the IMU orientation.", true));
  params_.publish_static_tf = declare_parameter<bool>(
    "publish_static_tf", true,
    describe("Broadcast the static <imu_frame> -> <cloud_frame> mounting offset.", true));
  params_.imu_to_lidar_translation = declare_parameter<std::vector<double>>(
    "imu_to_lidar_translation", params_.imu_to_lidar_translation,
    describe("Origin of the cloud frame in the IMU frame, in metres.", true));
  params_.imu_to_lidar_rotation = declare_parameter<std::vector<double>>(
    "imu_to_lidar_rotation", params_.imu_to_lidar_rotation,
    describe("Rotation from the IMU frame to the cloud frame as [x, y, z, w].", true));

  params_.start_rotation_on_startup = declare_parameter<bool>(
    "start_rotation_on_startup", true,
    describe("Send a start command on start up, needed when the lidar powers up in standby.",
      true));
  params_.stop_rotation_on_shutdown = declare_parameter<bool>(
    "stop_rotation_on_shutdown", false,
    describe("Stop the lidar rotating when the node shuts down.", true));

  params_.cloud_qos_profile = declare_parameter<std::string>(
    "cloud_qos_profile", "sensor_data",
    describe("Point cloud QoS: 'sensor_data' for best effort or 'default' for reliable.", true));
  params_.cloud_qos_depth = declare_parameter<int>(
    "cloud_qos_depth", 5, describeIntRange("Point cloud publisher queue depth.", 1, 10000, true));
  params_.imu_qos_profile = declare_parameter<std::string>(
    "imu_qos_profile", "sensor_data",
    describe("IMU QoS: 'sensor_data' for best effort or 'default' for reliable.", true));
  params_.imu_qos_depth = declare_parameter<int>(
    "imu_qos_depth", 5, describeIntRange("IMU publisher queue depth.", 1, 10000, true));
  params_.laserscan_qos_profile = declare_parameter<std::string>(
    "laserscan_qos_profile", "sensor_data",
    describe("LaserScan QoS: 'sensor_data' for best effort or 'default' for reliable.", true));
  params_.laserscan_qos_depth = declare_parameter<int>(
    "laserscan_qos_depth", 5,
    describeIntRange("LaserScan publisher queue depth.", 1, 10000, true));
  const std::string legacy_qos_profile = declare_parameter<std::string>(
    "qos_profile", "",
    describe(
      "Deprecated global QoS override. Empty uses the three per-topic profiles.", true));
  const int legacy_qos_depth = declare_parameter<int>(
    "qos_depth", 0,
    describeIntRange("Deprecated global QoS depth. 0 uses the per-topic depths.", 0, 10000, true));
  if (!legacy_qos_profile.empty()) {
    RCLCPP_WARN(
      get_logger(), "Parameter 'qos_profile' is deprecated; use the per-topic QoS profiles.");
    params_.cloud_qos_profile = legacy_qos_profile;
    params_.imu_qos_profile = legacy_qos_profile;
    params_.laserscan_qos_profile = legacy_qos_profile;
  }
  if (legacy_qos_depth > 0) {
    RCLCPP_WARN(
      get_logger(), "Parameter 'qos_depth' is deprecated; use the per-topic QoS depths.");
    params_.cloud_qos_depth = legacy_qos_depth;
    params_.imu_qos_depth = legacy_qos_depth;
    params_.laserscan_qos_depth = legacy_qos_depth;
  }

  params_.idle_sleep_us = declare_parameter<int>(
    "idle_sleep_us", 100,
    describeIntRange(
      "How long the reader thread sleeps when no packet was available, in microseconds. 0 spins.",
      0, 100000, true));
  params_.watchdog_timeout = declare_parameter<double>(
    "watchdog_timeout", 3.0,
    describeRange("Warn when no packet arrives for this many seconds. 0 disables.", 0.0, 3600.0,
      true));
  params_.diagnostics_topic = declare_parameter<std::string>(
    "diagnostics_topic", "/diagnostics",
    describe("Topic for diagnostic_msgs/DiagnosticArray health reports.", true));
  params_.diagnostics_period = declare_parameter<double>(
    "diagnostics_period", 1.0,
    describeRange("Diagnostic publication period in seconds. 0 disables.", 0.0, 3600.0, true));

  if (params_.range_max <= params_.range_min) {
    throw std::invalid_argument(
      "range_max (" + std::to_string(params_.range_max) + ") must be greater than range_min (" +
      std::to_string(params_.range_min) + ")");
  }
  if (params_.imu_to_lidar_translation.size() != 3) {
    throw std::invalid_argument(
      "imu_to_lidar_translation must have exactly 3 elements, got " +
      std::to_string(params_.imu_to_lidar_translation.size()));
  }
  if (params_.imu_to_lidar_rotation.size() != 4) {
    throw std::invalid_argument(
      "imu_to_lidar_rotation must have exactly 4 elements, got " +
      std::to_string(params_.imu_to_lidar_rotation.size()));
  }
  const double rotation_norm = std::sqrt(
    params_.imu_to_lidar_rotation[0] * params_.imu_to_lidar_rotation[0] +
    params_.imu_to_lidar_rotation[1] * params_.imu_to_lidar_rotation[1] +
    params_.imu_to_lidar_rotation[2] * params_.imu_to_lidar_rotation[2] +
    params_.imu_to_lidar_rotation[3] * params_.imu_to_lidar_rotation[3]);
  if (!std::isfinite(rotation_norm) || rotation_norm < kMinQuaternionNorm) {
    throw std::invalid_argument("imu_to_lidar_rotation must be a finite, non-zero quaternion");
  }
  for (double & component : params_.imu_to_lidar_rotation) {
    component /= rotation_norm;
  }

  if (params_.set_work_mode) {
    const uint32_t work_mode = static_cast<uint32_t>(params_.work_mode);
    const bool mode_selects_serial = (work_mode & kWorkModeSerialBit) != 0;
    const bool connected_over_serial = params_.connection == ConnectionType::Serial;
    if (mode_selects_serial != connected_over_serial) {
      const std::string message =
        "work_mode " + std::to_string(work_mode) + " selects " +
        (mode_selects_serial ? "serial" : "ethernet") + " but transport selects " +
        (connected_over_serial ? "serial" : "ethernet") +
        ". This persistent change moves the lidar to the other transport after restart.";
      if (!params_.allow_work_mode_transport_switch) {
        throw std::invalid_argument(
          message + " Set allow_work_mode_transport_switch:=true to confirm that change.");
      }
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }
}

void UnitreeLidarNode::createInterfaces()
{
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    params_.cloud_topic,
    makeQos(params_.cloud_qos_profile, params_.cloud_qos_depth), publisher_options);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
    params_.imu_topic,
    makeQos(params_.imu_qos_profile, params_.imu_qos_depth), publisher_options);
  pub_laserscan_ = create_publisher<sensor_msgs::msg::LaserScan>(
    params_.laserscan_topic,
    makeQos(params_.laserscan_qos_profile, params_.laserscan_qos_depth), publisher_options);
  if (params_.diagnostics_period > 0.0) {
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      params_.diagnostics_topic, rclcpp::QoS(10).reliable());
  }

  if (params_.publish_imu_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  if (params_.publish_static_tf) {
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }

  cloud_fields_ = buildCloudFields();
}

void UnitreeLidarNode::openLidar()
{
  // Resolve network configuration before allocating a reader. The vendor reader
  // cannot be deleted, so even a DNS or route error should not leak one.
  if (params_.connection == ConnectionType::Udp) {
    params_.lidar_ip = resolveIpv4Address(params_.lidar_ip, params_.lidar_port);
    if (params_.local_ip.empty() || params_.local_ip == "auto") {
      params_.local_ip = selectLocalIpv4Address(params_.lidar_ip, params_.lidar_port);
    } else {
      params_.local_ip = resolveIpv4Address(params_.local_ip, params_.local_port);
    }
  }

  unilidar_sdk2::UnitreeLidarReader * reader = unilidar_sdk2::createUnitreeLidarReader();
  if (reader == nullptr) {
    throw std::runtime_error("createUnitreeLidarReader() returned nullptr");
  }

  lidar_ = LidarReaderHandle(reader, LidarReaderRelease{});

  int result = -1;
  switch (params_.connection) {
    case ConnectionType::Serial:
      RCLCPP_INFO(
        get_logger(), "Opening lidar on serial port %s at %d baud",
        params_.serial_port.c_str(), params_.baudrate);
      result = lidar_->initializeSerial(
        params_.serial_port,
        static_cast<uint32_t>(params_.baudrate),
        static_cast<uint16_t>(params_.cloud_scan_num),
        params_.use_system_timestamp,
        static_cast<float>(params_.range_min),
        static_cast<float>(params_.range_max));
      if (result != 0) {
        throw std::runtime_error(
          "Failed to open serial port '" + params_.serial_port +
          "'. Check that the device exists, that this user can access it, and that the lidar is "
          "actually in serial mode.");
      }
      break;

    case ConnectionType::Udp:
      RCLCPP_INFO(
        get_logger(), "Opening lidar at %s:%d, listening on %s:%d",
        params_.lidar_ip.c_str(), params_.lidar_port,
        params_.local_ip.c_str(), params_.local_port);
      result = lidar_->initializeUDP(
        static_cast<uint16_t>(params_.lidar_port),
        params_.lidar_ip,
        static_cast<uint16_t>(params_.local_port),
        params_.local_ip,
        static_cast<uint16_t>(params_.cloud_scan_num),
        params_.use_system_timestamp,
        static_cast<float>(params_.range_min),
        static_cast<float>(params_.range_max));
      if (result != 0) {
        throw std::runtime_error(
          "Failed to open the UDP socket on " + params_.local_ip + ":" +
          std::to_string(params_.local_port) +
          ". Check that this host owns that address and that the port is free.");
      }
      break;

    default:
      throw std::invalid_argument(
        "Unknown connection type " +
        std::to_string(static_cast<int>(params_.connection)));
  }

  if (params_.set_work_mode) {
    lidar_->setLidarWorkMode(static_cast<uint32_t>(params_.work_mode));
  }

  if (params_.start_rotation_on_startup) {
    lidar_->startLidarRotation();
  }
}

void UnitreeLidarNode::publishStaticTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = params_.imu_frame;
  transform.child_frame_id = params_.cloud_frame;
  transform.transform.translation.x = params_.imu_to_lidar_translation[0];
  transform.transform.translation.y = params_.imu_to_lidar_translation[1];
  transform.transform.translation.z = params_.imu_to_lidar_translation[2];
  transform.transform.rotation.x = params_.imu_to_lidar_rotation[0];
  transform.transform.rotation.y = params_.imu_to_lidar_rotation[1];
  transform.transform.rotation.z = params_.imu_to_lidar_rotation[2];
  transform.transform.rotation.w = params_.imu_to_lidar_rotation[3];

  // This offset never changes, so it goes out once on a latched topic instead of
  // being re-sent with every IMU sample as it used to be.
  static_tf_broadcaster_->sendTransform(transform);
}

void UnitreeLidarNode::stopPolling() noexcept
{
  running_.store(false, std::memory_order_relaxed);
  if (poll_thread_.joinable()) {
    try {
      poll_thread_.join();
    } catch (const std::system_error &) {
      // Already gone.
    }
  }
}

void UnitreeLidarNode::pollLoop()
{
  const auto idle_sleep = std::chrono::microseconds(params_.idle_sleep_us);

  while (running_.load(std::memory_order_relaxed) && rclcpp::ok(context_)) {
    int packet_type = 0;
    try {
      packet_type = lidar_->runParse();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), steady_clock_, kThrottleMs, "Error reading from the lidar: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (packet_type == 0) {
      // Nothing buffered. Yield instead of spinning a core flat out.
      if (idle_sleep.count() > 0) {
        std::this_thread::sleep_for(idle_sleep);
      } else {
        std::this_thread::yield();
      }
      continue;
    }

    last_packet_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);

    // Publishing can throw if the context is shut down underneath us, and an
    // exception escaping this thread would terminate the process.
    try {
      switch (packet_type) {
        case LIDAR_IMU_DATA_PACKET_TYPE:
          handleImuPacket();
          break;
        case LIDAR_POINT_DATA_PACKET_TYPE:
          handlePointCloudPacket();
          break;
        case LIDAR_2D_POINT_DATA_PACKET_TYPE:
          handleLaserScanPacket();
          break;
        default:
          // Acknowledgements, version and config replies: consumed by the SDK.
          break;
      }
      reportVersionsOnce();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), steady_clock_, kThrottleMs, "Error publishing lidar data: %s", e.what());
    }
  }
}

void UnitreeLidarNode::handleImuPacket()
{
  // Braced: LidarImuData is a plain struct with no constructor.
  unilidar_sdk2::LidarImuData imu{};
  if (!lidar_->getImuData(imu)) {
    return;
  }

  // The SDK stores the quaternion as (x, y, z, w); see the print in the SDK's
  // own example. The transform broadcast below used to read it as (w, x, y, z),
  // which disagreed with the orientation published on the IMU topic.
  Orientation orientation;
  if (!normalizeOrientation(imu.quaternion, orientation)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "Dropping an IMU sample whose orientation quaternion is degenerate. Is the IMU disabled by "
      "bit 2 of work_mode?");
    return;
  }

  const rclcpp::Time stamp = resolveStamp(
    static_cast<double>(imu.info.stamp.sec) + static_cast<double>(imu.info.stamp.nsec) * 1e-9);

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = stamp;
  msg->header.frame_id = params_.imu_frame;

  msg->orientation.x = orientation.x;
  msg->orientation.y = orientation.y;
  msg->orientation.z = orientation.z;
  msg->orientation.w = orientation.w;

  msg->angular_velocity.x = imu.angular_velocity[0];
  msg->angular_velocity.y = imu.angular_velocity[1];
  msg->angular_velocity.z = imu.angular_velocity[2];

  msg->linear_acceleration.x = imu.linear_acceleration[0];
  msg->linear_acceleration.y = imu.linear_acceleration[1];
  msg->linear_acceleration.z = imu.linear_acceleration[2];

  pub_imu_->publish(std::move(msg));
  imu_count_.fetch_add(1, std::memory_order_relaxed);
  const int64_t published_ns = steadyNowNs();
  last_imu_time_ns_.store(published_ns, std::memory_order_relaxed);
  last_data_time_ns_.store(published_ns, std::memory_order_relaxed);

  if (params_.publish_imu_tf) {
    geometry_msgs::msg::TransformStamped transform;
    // Stamped with the sample's own time, not with now(): mixing the two makes
    // tf2 lookups on the cloud stamp fail with extrapolation errors.
    transform.header.stamp = stamp;
    transform.header.frame_id = params_.imu_frame + "_initial";
    transform.child_frame_id = params_.imu_frame;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = orientation.x;
    transform.transform.rotation.y = orientation.y;
    transform.transform.rotation.z = orientation.z;
    transform.transform.rotation.w = orientation.w;
    tf_broadcaster_->sendTransform(transform);
  }
}

void UnitreeLidarNode::handlePointCloudPacket()
{
  // Reuses the scratch cloud's capacity instead of allocating a fresh vector for
  // every scan.
  if (!lidar_->getPointCloud(cloud_scratch_)) {
    return;
  }

  const size_t num_points = cloud_scratch_.points.size();

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = resolveStamp(cloud_scratch_.stamp);
  msg->header.frame_id = params_.cloud_frame;
  msg->height = 1;
  msg->width = static_cast<uint32_t>(num_points);
  msg->fields = cloud_fields_;
  msg->is_bigendian = kIsBigEndian;
  msg->point_step = kPointStep;
  msg->row_step = static_cast<uint32_t>(kPointStep * num_points);
  msg->is_dense = true;
  msg->data.resize(kPointStep * num_points);

  packPointCloud(cloud_scratch_, msg->data.data());

  // Moved into the publisher so an intra process subscriber - a component in the
  // same container - gets the message without another copy of the payload.
  pub_cloud_->publish(std::move(msg));
  cloud_count_.fetch_add(1, std::memory_order_relaxed);
  const int64_t published_ns = steadyNowNs();
  last_cloud_time_ns_.store(published_ns, std::memory_order_relaxed);
  last_data_time_ns_.store(published_ns, std::memory_order_relaxed);
}

void UnitreeLidarNode::handleLaserScanPacket()
{
  const unilidar_sdk2::Lidar2DPointDataPacket & packet = lidar_->getLidar2DPointDataPacket();
  const unilidar_sdk2::Lidar2DPointData & data = packet.data;

  // point_num comes off the wire, so clamp it to the array it indexes.
  const size_t max_points = sizeof(data.ranges) / sizeof(data.ranges[0]);
  const size_t num_points = clampPointCount(data.point_num, max_points);
  if (num_points == 0) {
    return;
  }
  if (!validLaserScanMetadata(data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "Dropping a 2D scan with non-finite or invalid angle, timing, or range calibration data.");
    return;
  }

  const RangeWindow window = resolveRangeWindow(
    params_.range_min, params_.range_max, data.range_min, data.range_max);
  if (!window.valid()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "Dropping a 2D scan whose range window is empty (%.3f .. %.3f m).",
      window.min, window.max);
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp = resolveStamp(
    static_cast<double>(data.info.stamp.sec) + static_cast<double>(data.info.stamp.nsec) * 1e-9);
  msg->header.frame_id = params_.laserscan_frame;
  // The angle bias is part of the calibration, exactly as in the SDK's own 2D
  // parser; leaving it out skews the whole scan.
  msg->angle_min = data.angle_min + data.param.alpha_angle_bias;
  msg->angle_increment = data.angle_increment;
  msg->angle_max = scanAngleMax(msg->angle_min, data.angle_increment, num_points);
  msg->time_increment = data.time_increment;
  msg->scan_time = data.scan_period;
  msg->range_min = window.min;
  msg->range_max = window.max;
  msg->ranges.resize(num_points);
  msg->intensities.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    msg->ranges[i] = convertScanRange(data.ranges[i], data.param, window);
    msg->intensities[i] = static_cast<float>(data.intensities[i]);
  }

  pub_laserscan_->publish(std::move(msg));
  laserscan_count_.fetch_add(1, std::memory_order_relaxed);
  const int64_t published_ns = steadyNowNs();
  last_laserscan_time_ns_.store(published_ns, std::memory_order_relaxed);
  last_data_time_ns_.store(published_ns, std::memory_order_relaxed);
}

void UnitreeLidarNode::reportVersionsOnce()
{
  if (versions_reported_) {
    return;
  }

  std::string firmware;
  if (!lidar_->getVersionOfLidarFirmware(firmware)) {
    return;
  }

  std::string hardware;
  std::string sdk;
  lidar_->getVersionOfLidarHardware(hardware);
  lidar_->getVersionOfSDK(sdk);

  {
    std::lock_guard<std::mutex> lock(version_mutex_);
    firmware_version_ = firmware;
    hardware_version_ = hardware;
    sdk_version_ = sdk;
  }

  RCLCPP_INFO(
    get_logger(), "Lidar connected: hardware %s, firmware %s, sdk %s",
    hardware.c_str(), firmware.c_str(), sdk.c_str());
  versions_reported_ = true;
}

rclcpp::Time UnitreeLidarNode::resolveStamp(double sensor_stamp)
{
  if (timestamp_source_ == TimestampSource::Ros || !(sensor_stamp > 0.0)) {
    const rclcpp::Time now = this->now();
    if (use_sim_time_ && now.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, kThrottleMs,
        "use_sim_time is enabled but the ROS clock still reads zero; is anything publishing "
        "/clock?");
    }
    return now;
  }

  const rclcpp::Time stamp(sensorStampToNanoseconds(sensor_stamp), RCL_ROS_TIME);
  const double offset = (this->now() - stamp).seconds();
  if (std::fabs(offset) > kClockOffsetWarnSeconds) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "Lidar timestamps are %.3f s away from the ROS clock. Set use_system_timestamp:=true or "
      "timestamp_source:=ros if the lidar clock is not synchronised with this host.", offset);
  }
  return stamp;
}

void UnitreeLidarNode::checkLiveness()
{
  const int64_t now_ns = steadyNowNs();
  const uint64_t clouds = cloud_count_.load(std::memory_order_relaxed);
  const uint64_t imu_samples = imu_count_.load(std::memory_order_relaxed);
  const int64_t last = last_data_time_ns_.load(std::memory_order_relaxed);
  const double idle_seconds = static_cast<double>(now_ns - last) * 1e-9;
  if (idle_seconds < params_.watchdog_timeout) {
    return;
  }

  if (params_.connection == ConnectionType::Udp) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "No valid sensor data for %.1f s (%" PRIu64 " clouds and %" PRIu64
      " imu samples so far). "
      "Check that it is powered and rotating, that %s is reachable, and that this host is "
      "configured as %s:%d.",
      idle_seconds, clouds, imu_samples,
      params_.lidar_ip.c_str(), params_.local_ip.c_str(), params_.local_port);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kThrottleMs,
      "No valid sensor data for %.1f s on %s (%" PRIu64 " clouds and %" PRIu64
      " imu samples so far). Check that it is powered, rotating, and in serial mode.",
      idle_seconds, params_.serial_port.c_str(), clouds, imu_samples);
  }
}

void UnitreeLidarNode::publishDiagnostics()
{
  if (!pub_diagnostics_) {
    return;
  }

  const int64_t now_ns = steadyNowNs();
  const uint64_t clouds = cloud_count_.load(std::memory_order_relaxed);
  const uint64_t imu_samples = imu_count_.load(std::memory_order_relaxed);
  const uint64_t scans = laserscan_count_.load(std::memory_order_relaxed);
  const double elapsed = last_rate_report_ns_ > 0 ?
    static_cast<double>(now_ns - last_rate_report_ns_) * 1e-9 : 0.0;
  const double cloud_rate = elapsed > 0.0 ?
    static_cast<double>(clouds - reported_cloud_count_) / elapsed : 0.0;
  const double imu_rate = elapsed > 0.0 ?
    static_cast<double>(imu_samples - reported_imu_count_) / elapsed : 0.0;
  const double scan_rate = elapsed > 0.0 ?
    static_cast<double>(scans - reported_laserscan_count_) / elapsed : 0.0;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_fully_qualified_name()) + ": Unitree L2 driver";

  {
    std::lock_guard<std::mutex> lock(version_mutex_);
    status.hardware_id = hardware_version_.empty() ? "unitree_l2" : hardware_version_;
    status.values.push_back(diagnosticValue("hardware_version", hardware_version_));
    status.values.push_back(diagnosticValue("firmware_version", firmware_version_));
    status.values.push_back(diagnosticValue("sdk_version", sdk_version_));
  }

  const int64_t last_data = last_data_time_ns_.load(std::memory_order_relaxed);
  const double data_age = static_cast<double>(now_ns - last_data) * 1e-9;
  const double stale_timeout = params_.watchdog_timeout > 0.0 ?
    params_.watchdog_timeout : std::max(3.0, 3.0 * params_.diagnostics_period);
  const bool has_data = clouds > 0 || imu_samples > 0 || scans > 0;
  if (data_age >= stale_timeout) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "No valid sensor data";
  } else if (!has_data) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Waiting for sensor data";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Streaming";
  }

  status.values.push_back(diagnosticValue(
    "transport", params_.connection == ConnectionType::Serial ? "serial" : "ethernet"));
  status.values.push_back(diagnosticValue(
    "endpoint", params_.connection == ConnectionType::Serial ? params_.serial_port :
    params_.lidar_ip + ":" + std::to_string(params_.lidar_port)));
  status.values.push_back(diagnosticValue(
    "last_packet_age_sec", formatAge(
      now_ns, last_packet_time_ns_.load(std::memory_order_relaxed))));
  status.values.push_back(diagnosticValue("last_data_age_sec", formatAge(now_ns, last_data)));
  status.values.push_back(diagnosticValue(
    "last_cloud_age_sec", formatAge(
      now_ns, last_cloud_time_ns_.load(std::memory_order_relaxed))));
  status.values.push_back(diagnosticValue(
    "last_imu_age_sec", formatAge(now_ns, last_imu_time_ns_.load(std::memory_order_relaxed))));
  status.values.push_back(diagnosticValue(
    "last_scan_age_sec", formatAge(
      now_ns, last_laserscan_time_ns_.load(std::memory_order_relaxed))));
  status.values.push_back(diagnosticValue("cloud_count", std::to_string(clouds)));
  status.values.push_back(diagnosticValue("cloud_rate_hz", formatRate(cloud_rate)));
  status.values.push_back(diagnosticValue("imu_count", std::to_string(imu_samples)));
  status.values.push_back(diagnosticValue("imu_rate_hz", formatRate(imu_rate)));
  status.values.push_back(diagnosticValue("scan_count", std::to_string(scans)));
  status.values.push_back(diagnosticValue("scan_rate_hz", formatRate(scan_rate)));

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = this->now();
  message.status.push_back(std::move(status));
  pub_diagnostics_->publish(std::move(message));

  RCLCPP_DEBUG(
    get_logger(), "cloud %.1f Hz, imu %.1f Hz, 2d scan %.1f Hz",
    cloud_rate, imu_rate, scan_rate);
  last_rate_report_ns_ = now_ns;
  reported_cloud_count_ = clouds;
  reported_imu_count_ = imu_samples;
  reported_laserscan_count_ = scans;
}

}  // namespace unitree_lidar_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(unitree_lidar_ros2::UnitreeLidarNode)
