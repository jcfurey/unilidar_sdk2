/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// Drives the real node with synthesised UDP packets and checks what it publishes.
// No lidar involved: the driver binds to loopback and the test plays the part of
// the sensor.
//
// The message stamps are taken from the node clock (timestamp_source "ros"). The
// SDK's own cloud stamp bookkeeping does not settle from a single synthetic scan
// line - it leaves PointCloudUnitree::stamp at zero - so asserting on it here
// would be testing an artefact of the fixture rather than the driver.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "packet_builder.hpp"
#include "unitree_lidar_ros2/point_cloud_layout.hpp"
#include "unitree_lidar_ros2/unitree_lidar_node.hpp"

namespace
{

using namespace std::chrono_literals;

// Away from the driver's defaults so a stray lidar or a second test cannot
// interfere.
constexpr uint16_t kFirstLocalPort = 16201;
constexpr uint16_t kFirstLidarPort = 16101;

class DriverFixture : public testing::Test
{
protected:
  /// Each test gets its own port pair.
  ///
  /// Not fastidiousness: the SDK cannot release a connection at all. closeUDP()
  /// crashes on an open connection and UnitreeLidarReader has no virtual
  /// destructor, so a driver holds its socket until the process exits and a second
  /// instance on the same port fails to bind. See LidarReaderRelease.
  static uint16_t nextPortOffset()
  {
    static uint16_t offset = 0;
    return offset++;
  }

  void SetUp() override
  {
    const uint16_t offset = nextPortOffset();
    local_port_ = static_cast<uint16_t>(kFirstLocalPort + offset);
    lidar_port_ = static_cast<uint16_t>(kFirstLidarPort + offset);

    const std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("transport", "ethernet"),
      // Exercise the DHCP-compatible path: resolve the sensor hostname and
      // derive the local bind address from the route to it.
      rclcpp::Parameter("local_ip", "auto"),
      rclcpp::Parameter("local_port", static_cast<int>(local_port_)),
      rclcpp::Parameter("lidar_ip", "localhost"),
      rclcpp::Parameter("lidar_port", static_cast<int>(lidar_port_)),
      // One scan line per cloud, so a single packet produces a message.
      rclcpp::Parameter("cloud_scan_num", 1),
      rclcpp::Parameter("timestamp_source", std::string("ros")),
      rclcpp::Parameter("set_work_mode", false),
      rclcpp::Parameter("start_rotation_on_startup", false),
      rclcpp::Parameter("imu_to_lidar_translation", std::vector<double>{0.5, -0.25, 0.125}),
      rclcpp::Parameter(
        "imu_to_lidar_rotation", std::vector<double>{0.0, 0.0, 0.70710678, 0.70710678}),
      rclcpp::Parameter("watchdog_timeout", 0.0),
      rclcpp::Parameter("diagnostics_period", 0.1),
    };

    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);

    driver_ = std::make_shared<unitree_lidar_ros2::UnitreeLidarNode>(options);
    listener_ = std::make_shared<rclcpp::Node>("e2e_listener_" + std::to_string(offset));
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(driver_);
    executor_->add_node(listener_);

    imu_subscription_ = listener_->create_subscription<sensor_msgs::msg::Imu>(
      "unilidar/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {imu_messages_.push_back(*msg);});
    cloud_subscription_ = listener_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "unilidar/cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {cloud_messages_.push_back(*msg);});
    diagnostics_subscription_ =
      listener_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        diagnostic_messages_.push_back(*msg);
      });
    tf_subscription_ = listener_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {tf_messages_.push_back(*msg);});
    // Static transforms are latched, so the subscription has to be too.
    static_tf_subscription_ = listener_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).transient_local(),
      [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {static_tf_messages_.push_back(*msg);});

    sender_ = std::make_unique<unitree_lidar_ros2::test::PacketSender>("127.0.0.1", local_port_);
    ASSERT_TRUE(sender_->valid());
  }

  void TearDown() override
  {
    // Drop the driver first so its reader thread is joined while the context lives.
    if (executor_ && driver_) {
      executor_->remove_node(driver_);
    }
    driver_.reset();
    if (executor_ && listener_) {
      executor_->remove_node(listener_);
    }
    executor_.reset();
    listener_.reset();
  }

  /// Spins the listener until @p ready holds, or the deadline passes.
  template<typename PredicateT>
  bool spinUntil(PredicateT ready, std::chrono::milliseconds timeout = 8000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (ready()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    executor_->spin_some();
    return ready();
  }

  uint16_t local_port_{0};
  uint16_t lidar_port_{0};
  std::shared_ptr<unitree_lidar_ros2::UnitreeLidarNode> driver_;
  std::shared_ptr<rclcpp::Node> listener_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<unitree_lidar_ros2::test::PacketSender> sender_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_subscription_;

  std::vector<sensor_msgs::msg::Imu> imu_messages_;
  std::vector<sensor_msgs::msg::PointCloud2> cloud_messages_;
  std::vector<diagnostic_msgs::msg::DiagnosticArray> diagnostic_messages_;
  std::vector<tf2_msgs::msg::TFMessage> tf_messages_;
  std::vector<tf2_msgs::msg::TFMessage> static_tf_messages_;
};

/// Reads a value out of a PointCloud2 payload without assuming any alignment.
template<typename T>
T readField(const sensor_msgs::msg::PointCloud2 & cloud, size_t point, size_t offset)
{
  T value{};
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(T));
  return value;
}

}  // namespace

// The orientation on the topic and the orientation in the transform must agree.
// They did not: the message read the SDK array as (x, y, z, w) while the
// transform read the same array as (w, x, y, z).
TEST_F(DriverFixture, PublishesImuWithTheSdkQuaternionOrder)
{
  auto packet = unitree_lidar_ros2::test::makeImuPacket();
  ASSERT_TRUE(sender_->send(packet, LIDAR_IMU_DATA_PACKET_TYPE));

  ASSERT_TRUE(spinUntil([this] {return !imu_messages_.empty() && !tf_messages_.empty();}))
    << "no IMU message and transform arrived";

  const auto & imu = imu_messages_.front();
  EXPECT_EQ(imu.header.frame_id, "unilidar_imu");
  EXPECT_NEAR(imu.orientation.x, 0.1, 1e-6);
  EXPECT_NEAR(imu.orientation.y, 0.2, 1e-6);
  EXPECT_NEAR(imu.orientation.z, 0.3, 1e-6);
  EXPECT_NEAR(imu.orientation.w, 0.9273618, 1e-6);

  EXPECT_NEAR(imu.angular_velocity.x, 1.5, 1e-6);
  EXPECT_NEAR(imu.angular_velocity.y, -2.5, 1e-6);
  EXPECT_NEAR(imu.angular_velocity.z, 0.25, 1e-6);

  EXPECT_NEAR(imu.linear_acceleration.x, 0.11, 1e-6);
  EXPECT_NEAR(imu.linear_acceleration.y, 0.22, 1e-6);
  EXPECT_NEAR(imu.linear_acceleration.z, 9.81, 1e-6);

  ASSERT_FALSE(tf_messages_.front().transforms.empty());
  const auto & transform = tf_messages_.front().transforms.front();
  EXPECT_EQ(transform.header.frame_id, "unilidar_imu_initial");
  EXPECT_EQ(transform.child_frame_id, "unilidar_imu");
  EXPECT_NEAR(transform.transform.rotation.x, imu.orientation.x, 1e-9);
  EXPECT_NEAR(transform.transform.rotation.y, imu.orientation.y, 1e-9);
  EXPECT_NEAR(transform.transform.rotation.z, imu.orientation.z, 1e-9);
  EXPECT_NEAR(transform.transform.rotation.w, imu.orientation.w, 1e-9);

  // The transform must carry the sample's stamp, not now(): mixing the two makes
  // tf2 lookups on the cloud stamp fail with extrapolation errors.
  EXPECT_EQ(rclcpp::Time(transform.header.stamp), rclcpp::Time(imu.header.stamp));
}

TEST_F(DriverFixture, PublishesAPclCompatiblePointCloud)
{
  auto packet = unitree_lidar_ros2::test::makePointPacket(5);
  ASSERT_TRUE(sender_->send(packet, LIDAR_POINT_DATA_PACKET_TYPE));

  ASSERT_TRUE(spinUntil([this] {return !cloud_messages_.empty();})) << "no cloud arrived";

  const auto & cloud = cloud_messages_.front();
  EXPECT_EQ(cloud.header.frame_id, "unilidar_lidar");
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.width, 5u);
  EXPECT_EQ(cloud.point_step, unitree_lidar_ros2::kPointStep);
  EXPECT_EQ(cloud.row_step, unitree_lidar_ros2::kPointStep * 5);
  EXPECT_EQ(cloud.data.size(), unitree_lidar_ros2::kPointStep * 5);
  EXPECT_TRUE(cloud.is_dense);

  // The layout consumers are configured against, unchanged from the PCL based
  // implementation this replaced.
  ASSERT_EQ(cloud.fields.size(), 6u);
  const std::vector<std::string> names{"x", "y", "z", "intensity", "ring", "time"};
  const std::vector<uint32_t> offsets{0, 4, 8, 16, 20, 24};
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(cloud.fields[i].name, names[i]);
    EXPECT_EQ(cloud.fields[i].offset, offsets[i]);
    EXPECT_EQ(cloud.fields[i].count, 1u);
  }
  EXPECT_EQ(cloud.fields[4].datatype, sensor_msgs::msg::PointField::UINT16);

  // Identity calibration with both sweeps held still puts the returns on the +Y
  // axis at exactly 1 m .. 5 m, with intensities 10 .. 50.
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(readField<float>(cloud, i, 0), 0.0f, 1e-6f) << "point " << i << " x";
    EXPECT_NEAR(readField<float>(cloud, i, 4), static_cast<float>(i + 1), 1e-5f)
      << "point " << i << " y";
    EXPECT_NEAR(readField<float>(cloud, i, 8), 0.0f, 1e-6f) << "point " << i << " z";
    EXPECT_NEAR(readField<float>(cloud, i, 16), static_cast<float>(10 * (i + 1)), 1e-6f)
      << "point " << i << " intensity";
  }
}

TEST_F(DriverFixture, PublishesStandardDiagnosticsForValidData)
{
  auto packet = unitree_lidar_ros2::test::makeImuPacket();
  ASSERT_TRUE(sender_->send(packet, LIDAR_IMU_DATA_PACKET_TYPE));

  ASSERT_TRUE(spinUntil([this] {
      for (const auto & message : diagnostic_messages_) {
        if (!message.status.empty() &&
        message.status.front().level == diagnostic_msgs::msg::DiagnosticStatus::OK)
        {
          return true;
        }
      }
      return false;
    })) << "no healthy diagnostic report arrived";

  const auto & status = diagnostic_messages_.back().status.front();
  EXPECT_EQ(status.message, "Streaming");
  EXPECT_EQ(status.hardware_id, "unitree_l2");
  EXPECT_FALSE(status.values.empty());
}

TEST_F(DriverFixture, PublishesTheMountingOffsetOnceAsAStaticTransform)
{
  ASSERT_TRUE(spinUntil([this] {return !static_tf_messages_.empty();}))
    << "the static transform was not latched";

  ASSERT_FALSE(static_tf_messages_.front().transforms.empty());
  const auto & transform = static_tf_messages_.front().transforms.front();
  EXPECT_EQ(transform.header.frame_id, "unilidar_imu");
  EXPECT_EQ(transform.child_frame_id, "unilidar_lidar");
  EXPECT_NEAR(transform.transform.translation.x, 0.5, 1e-9);
  EXPECT_NEAR(transform.transform.translation.y, -0.25, 1e-9);
  EXPECT_NEAR(transform.transform.translation.z, 0.125, 1e-9);
  EXPECT_NEAR(transform.transform.rotation.z, 0.70710678, 1e-8);
  EXPECT_NEAR(transform.transform.rotation.w, 0.70710678, 1e-8);
}

// All zeros is what the lidar sends when its IMU is disabled by work_mode bit 2.
// tf2 rejects such a quaternion, so the driver drops the sample instead.
TEST_F(DriverFixture, DropsImuSamplesWithADegenerateQuaternion)
{
  auto packet = unitree_lidar_ros2::test::makeImuPacket();
  for (float & component : packet.data.quaternion) {
    component = 0.0f;
  }
  ASSERT_TRUE(sender_->send(packet, LIDAR_IMU_DATA_PACKET_TYPE));

  EXPECT_FALSE(spinUntil([this] {return !imu_messages_.empty();}, 2500ms))
    << "a degenerate orientation was published anyway";
}

// A packet claiming far more points than the array can hold must not produce a
// cloud bigger than the array, nor read past the end of it.
TEST_F(DriverFixture, SurvivesAPacketWithAnOversizedPointCount)
{
  auto packet = unitree_lidar_ros2::test::makePointPacket(300);
  packet.data.point_num = 0xFFFFFFFFu;
  ASSERT_TRUE(sender_->send(packet, LIDAR_POINT_DATA_PACKET_TYPE));

  // Either a bounded cloud or nothing at all is acceptable; a crash is not.
  spinUntil([this] {return !cloud_messages_.empty();}, 3000ms);
  for (const auto & cloud : cloud_messages_) {
    EXPECT_LE(cloud.width, 300u);
    EXPECT_EQ(cloud.data.size(), cloud.width * unitree_lidar_ros2::kPointStep);
  }

  // The node must still be healthy afterwards.
  auto imu_packet = unitree_lidar_ros2::test::makeImuPacket();
  ASSERT_TRUE(sender_->send(imu_packet, LIDAR_IMU_DATA_PACKET_TYPE));
  EXPECT_TRUE(spinUntil([this] {return !imu_messages_.empty();}))
    << "the driver stopped publishing after the malformed packet";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
