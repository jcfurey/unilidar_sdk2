/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "unitree_lidar_ros2/unitree_lidar_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    // Scoped so the node - and with it the reader thread and the lidar
    // connection - is torn down before the context goes away.
    auto node = std::make_shared<unitree_lidar_ros2::UnitreeLidarNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // A bad parameter or an unreachable lidar should report what went wrong and
    // exit non-zero, rather than letting the exception abort the process.
    RCLCPP_FATAL(
      rclcpp::get_logger("unitree_lidar_ros2"), "Could not start the lidar driver: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
