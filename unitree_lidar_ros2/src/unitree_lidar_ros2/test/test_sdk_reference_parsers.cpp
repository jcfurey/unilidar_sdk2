/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// Covers the reference parsers in unitree_lidar_utilities.h, which are what the
// SDK documentation points users at for decoding raw packets themselves.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "unitree_lidar_utilities.h"  // NOLINT(build/include_subdir)

namespace
{

constexpr size_t k3dCapacity =
  sizeof(unilidar_sdk2::LidarPointData::ranges) / sizeof(uint16_t);
constexpr size_t k2dCapacity =
  sizeof(unilidar_sdk2::Lidar2DPointData::ranges) / sizeof(uint16_t);

/// A packet with identity calibration, so returns land on the +Y axis at
/// (0, range, 0) and the arithmetic stays easy to check by hand.
unilidar_sdk2::LidarPointDataPacket makeIdentityPacket(uint32_t point_num)
{
  unilidar_sdk2::LidarPointDataPacket packet{};
  packet.data.point_num = point_num;
  packet.data.range_min = 0.0f;
  packet.data.range_max = 1000.0f;
  packet.data.angle_min = 0.0f;
  packet.data.angle_increment = 0.0f;
  packet.data.com_horizontal_angle_start = 0.0f;
  packet.data.com_horizontal_angle_step = 0.0f;
  packet.data.time_increment = 0.0f;
  packet.data.scan_period = 0.0f;
  packet.data.param.range_scale = 0.001f;
  packet.data.param.range_bias = 0.0f;

  const size_t filled = point_num < k3dCapacity ? point_num : k3dCapacity;
  for (size_t i = 0; i < filled; ++i) {
    packet.data.ranges[i] = static_cast<uint16_t>(1000 * (i + 1));
    packet.data.intensities[i] = static_cast<uint8_t>(i + 1);
  }
  return packet;
}

unilidar_sdk2::Lidar2DPointDataPacket make2dPacket(uint32_t point_num)
{
  unilidar_sdk2::Lidar2DPointDataPacket packet{};
  packet.data.point_num = point_num;
  packet.data.range_min = 0.0f;
  packet.data.range_max = 1000.0f;
  packet.data.angle_min = 0.0f;
  packet.data.angle_increment = 0.0f;
  packet.data.time_increment = 0.0f;
  packet.data.scan_period = 0.0f;
  packet.data.param.range_scale = 0.001f;
  packet.data.param.range_bias = 0.0f;

  const size_t filled = point_num < k2dCapacity ? point_num : k2dCapacity;
  for (size_t i = 0; i < filled; ++i) {
    packet.data.ranges[i] = static_cast<uint16_t>(1000 * (i + 1));
  }
  return packet;
}

}  // namespace

TEST(ReferenceParser3d, DecodesReturnsWithIdentityCalibration)
{
  const auto packet = makeIdentityPacket(3);
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);

  ASSERT_EQ(cloud.points.size(), 3u);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    EXPECT_NEAR(cloud.points[i].x, 0.0f, 1e-6) << "point " << i;
    EXPECT_NEAR(cloud.points[i].y, static_cast<float>(i + 1), 1e-5) << "point " << i;
    EXPECT_NEAR(cloud.points[i].z, 0.0f, 1e-6) << "point " << i;
    EXPECT_FLOAT_EQ(cloud.points[i].intensity, static_cast<float>(i + 1));
  }
}

// point_num arrives inside the packet. Before this was clamped, a corrupted or
// truncated packet made the loop read past the end of the ranges array.
TEST(ReferenceParser3d, ClampsAnOversizedPointCountFromTheWire)
{
  for (const uint32_t point_num : {static_cast<uint32_t>(k3dCapacity + 1), 100000u, 0xFFFFFFFFu}) {
    auto packet = makeIdentityPacket(point_num);
    // Fill the whole array so every in-bounds entry is a valid return; anything
    // beyond the array would have to come from out of bounds memory.
    for (size_t i = 0; i < k3dCapacity; ++i) {
      packet.data.ranges[i] = 1000;
    }
    unilidar_sdk2::PointCloudUnitree cloud{};
    unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);
    EXPECT_LE(cloud.points.size(), k3dCapacity) << "point_num = " << point_num;
    EXPECT_EQ(cloud.points.size(), k3dCapacity) << "point_num = " << point_num;
  }
}

TEST(ReferenceParser3d, SkipsReturnsWithNoEcho)
{
  auto packet = makeIdentityPacket(4);
  packet.data.ranges[1] = 0;  // no echo
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);
  EXPECT_EQ(cloud.points.size(), 3u);
}

TEST(ReferenceParser3d, HonoursTheCallersRangeLimits)
{
  const auto packet = makeIdentityPacket(5);  // returns at 1..5 m
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 2.0f, 4.0f);
  ASSERT_EQ(cloud.points.size(), 3u);
  EXPECT_NEAR(cloud.points.front().y, 2.0f, 1e-5);
  EXPECT_NEAR(cloud.points.back().y, 4.0f, 1e-5);
}

TEST(ReferenceParser3d, TreatsAnEmptyPacketRangeAsUnspecified)
{
  auto packet = makeIdentityPacket(3);
  packet.data.range_min = 0.0f;
  packet.data.range_max = 0.0f;
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);
  EXPECT_EQ(cloud.points.size(), 3u);
}

TEST(ReferenceParser3d, StartsFromAnEmptyCloudOnEveryCall)
{
  const auto packet = makeIdentityPacket(2);
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);
  unilidar_sdk2::parseFromPacketToPointCloud(cloud, packet, false, 0.0f, 1000.0f);
  EXPECT_EQ(cloud.points.size(), 2u) << "the cloud must be cleared, not appended to";
}

TEST(ReferenceParser2d, ClampsAnOversizedPointCountFromTheWire)
{
  for (const uint32_t point_num : {static_cast<uint32_t>(k2dCapacity + 1), 0xFFFFFFFFu}) {
    auto packet = make2dPacket(point_num);
    for (size_t i = 0; i < k2dCapacity; ++i) {
      packet.data.ranges[i] = 1000;
    }
    unilidar_sdk2::PointCloudUnitree cloud{};
    unilidar_sdk2::parseFromPacketPointCloud2D(cloud, packet, false, 0.0f, 1000.0f);
    EXPECT_EQ(cloud.points.size(), k2dCapacity) << "point_num = " << point_num;
  }
}

TEST(ReferenceParser2d, PlacesReturnsInTheYzPlane)
{
  const auto packet = make2dPacket(2);
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketPointCloud2D(cloud, packet, false, 0.0f, 1000.0f);
  ASSERT_EQ(cloud.points.size(), 2u);
  for (const auto & point : cloud.points) {
    EXPECT_FLOAT_EQ(point.x, 0.0f) << "the 2D scan is a planar sweep";
  }
  EXPECT_NEAR(cloud.points[0].y, 1.0f, 1e-5);
  EXPECT_NEAR(cloud.points[1].y, 2.0f, 1e-5);
}

TEST(ReferenceParser2d, TreatsAnEmptyPacketRangeAsUnspecified)
{
  auto packet = make2dPacket(3);
  packet.data.range_min = 0.0f;
  packet.data.range_max = 0.0f;
  unilidar_sdk2::PointCloudUnitree cloud{};
  unilidar_sdk2::parseFromPacketPointCloud2D(cloud, packet, false, 0.0f, 1000.0f);
  EXPECT_EQ(cloud.points.size(), 3u);
}

// The protocol structs are overlaid on received bytes, so their layout is part of
// the wire format. unitree_lidar_protocol.h asserts this at compile time; this
// keeps the expectation visible in the test report too.
TEST(ProtocolLayout, MatchesTheDocumentedSizes)
{
  EXPECT_EQ(sizeof(unilidar_sdk2::LidarPointDataPacket), 1044u);
  EXPECT_EQ(sizeof(unilidar_sdk2::Lidar2DPointDataPacket), 5536u);
  EXPECT_EQ(sizeof(unilidar_sdk2::LidarImuDataPacket), 80u);
  EXPECT_EQ(k3dCapacity, 300u);
  EXPECT_EQ(k2dCapacity, 1800u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
