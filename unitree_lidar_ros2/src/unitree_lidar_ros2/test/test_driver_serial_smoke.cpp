/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "unitree_lidar_ros2/unitree_lidar_node.hpp"

namespace
{
using namespace std::chrono_literals;

TEST(PersistentConfiguration, RejectsAnUnconfirmedTransportSwitchBeforeOpeningHardware)
{
  const std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("transport", "serial"),
    rclcpp::Parameter("set_work_mode", true),
    // Bit 3 is clear, so this would persistently switch the lidar to ethernet.
    rclcpp::Parameter("work_mode", 0),
  };
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);

  EXPECT_THROW(
    std::make_shared<unitree_lidar_ros2::UnitreeLidarNode>(options), std::invalid_argument);
}

TEST(SerialTransport, OpensFourMegabaudPseudoTerminalAndStopsCleanly)
{
  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master_fd, 0);
  ASSERT_EQ(grantpt(master_fd), 0);
  ASSERT_EQ(unlockpt(master_fd), 0);
  const char * slave_name = ptsname(master_fd);
  ASSERT_NE(slave_name, nullptr);

  const std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("transport", "serial"),
    rclcpp::Parameter("serial_port", std::string(slave_name)),
    rclcpp::Parameter("baudrate", 4000000),
    rclcpp::Parameter("set_work_mode", false),
    rclcpp::Parameter("start_rotation_on_startup", false),
    rclcpp::Parameter("publish_imu_tf", false),
    rclcpp::Parameter("publish_static_tf", false),
    rclcpp::Parameter("watchdog_timeout", 0.0),
    rclcpp::Parameter("diagnostics_period", 0.0),
  };
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);

  auto driver = std::make_shared<unitree_lidar_ros2::UnitreeLidarNode>(options);
  std::this_thread::sleep_for(100ms);
  driver.reset();

  EXPECT_EQ(close(master_fd), 0);
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
