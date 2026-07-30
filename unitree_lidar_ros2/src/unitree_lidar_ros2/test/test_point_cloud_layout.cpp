/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "unitree_lidar_ros2/point_cloud_layout.hpp"

namespace
{

/// Reads a value back out of the packed buffer without assuming the buffer is
/// aligned for it, the way a subscriber walking the cloud would.
template<typename T>
T readAt(const std::vector<uint8_t> & data, size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

unilidar_sdk2::PointUnitree makePoint(
  float x, float y, float z, float intensity, float time,
  uint32_t ring)
{
  unilidar_sdk2::PointUnitree point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.time = time;
  point.ring = ring;
  return point;
}

}  // namespace

// The published cloud has to stay binary compatible with what pcl::toROSMsg()
// used to emit for unitree_lidar_sdk_pcl.h's PointType, because downstream
// consumers are configured with these field names and offsets.
TEST(PointCloudLayout, MatchesPclPointType)
{
  EXPECT_EQ(unitree_lidar_ros2::kPointStep, 32u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, x), 0u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, y), 4u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, z), 8u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, intensity), 16u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, ring), 20u);
  EXPECT_EQ(offsetof(unitree_lidar_ros2::PackedPoint, time), 24u);
}

TEST(PointCloudLayout, PacksEveryFieldAtItsOffset)
{
  unilidar_sdk2::PointCloudUnitree cloud;
  cloud.points.push_back(makePoint(1.5f, -2.25f, 0.0f, 42.0f, 0.0f, 1));
  cloud.points.push_back(makePoint(-100.125f, 3.75f, 17.5f, 255.0f, 7.8e-6f, 18));

  std::vector<uint8_t> data(cloud.points.size() * unitree_lidar_ros2::kPointStep, 0);
  unitree_lidar_ros2::packPointCloud(cloud, data.data());

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const size_t base = i * unitree_lidar_ros2::kPointStep;
    const unilidar_sdk2::PointUnitree & expected = cloud.points[i];

    EXPECT_FLOAT_EQ(readAt<float>(data, base + 0), expected.x) << "point " << i;
    EXPECT_FLOAT_EQ(readAt<float>(data, base + 4), expected.y) << "point " << i;
    EXPECT_FLOAT_EQ(readAt<float>(data, base + 8), expected.z) << "point " << i;
    // PCL's homogeneous coordinate.
    EXPECT_FLOAT_EQ(readAt<float>(data, base + 12), 1.0f) << "point " << i;
    EXPECT_FLOAT_EQ(readAt<float>(data, base + 16), expected.intensity) << "point " << i;
    EXPECT_EQ(readAt<uint16_t>(data, base + 20),
      static_cast<uint16_t>(expected.ring)) << "point " << i;
    EXPECT_FLOAT_EQ(readAt<float>(data, base + 24), expected.time) << "point " << i;
  }
}

TEST(PointCloudLayout, HandlesAnEmptyCloud)
{
  const unilidar_sdk2::PointCloudUnitree cloud{};
  std::vector<uint8_t> data;
  // Must not touch the (null) buffer of an empty vector.
  unitree_lidar_ros2::packPointCloud(cloud, data.data());
  EXPECT_TRUE(data.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
