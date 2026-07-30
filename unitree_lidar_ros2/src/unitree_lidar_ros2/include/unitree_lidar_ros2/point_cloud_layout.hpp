/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef UNITREE_LIDAR_ROS2__POINT_CLOUD_LAYOUT_HPP_
#define UNITREE_LIDAR_ROS2__POINT_CLOUD_LAYOUT_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "unitree_lidar_sdk.h"  // NOLINT(build/include_subdir)

namespace unitree_lidar_ros2
{

/**
 * @brief Binary layout of a single point in the published sensor_msgs/PointCloud2.
 *
 * This reproduces, byte for byte, what pcl::toROSMsg() emits for the
 * `PointType` declared in unitree_lidar_sdk_pcl.h, which is what this driver
 * used to publish:
 *
 *   x @0  y @4  z @8  (pad @12)  intensity @16  ring @20  (pad @22)  time @24
 *   point_step = 32
 *
 * Keeping the layout identical means existing consumers - SLAM front ends in
 * particular, which are usually configured with hard coded field names and
 * offsets - keep working, while the driver itself no longer has to depend on
 * PCL or copy every cloud through a pcl::PointCloud.
 *
 * The padding is part of the layout, not an accident: PCL's point types are
 * 16 byte aligned and reserve a fourth float after z.
 */
struct PackedPoint
{
  float x;
  float y;
  float z;
  /// Homogeneous coordinate PCL keeps after z. PCL sets it to 1 and some of its
  /// Eigen based algorithms rely on that, so it is filled in rather than left
  /// as padding.
  float w;
  float intensity;
  uint16_t ring;
  uint16_t reserved;
  float time;
  float padding;
};

static_assert(sizeof(PackedPoint) == 32, "PackedPoint must match pcl::PointCloud<PointType>");
static_assert(offsetof(PackedPoint, x) == 0, "unexpected x offset");
static_assert(offsetof(PackedPoint, y) == 4, "unexpected y offset");
static_assert(offsetof(PackedPoint, z) == 8, "unexpected z offset");
static_assert(offsetof(PackedPoint, intensity) == 16, "unexpected intensity offset");
static_assert(offsetof(PackedPoint, ring) == 20, "unexpected ring offset");
static_assert(offsetof(PackedPoint, time) == 24, "unexpected time offset");

/// Size of one point in the published cloud, in bytes.
constexpr uint32_t kPointStep = static_cast<uint32_t>(sizeof(PackedPoint));

/// Points are written in host byte order, which is what `is_bigendian` reports.
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__)
constexpr bool kIsBigEndian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
#else
constexpr bool kIsBigEndian = false;
#endif

/**
 * @brief Serialises @p cloud into the PointCloud2 payload layout.
 *
 * @param cloud Cloud as handed over by the SDK.
 * @param dst   Destination buffer, at least cloud.points.size() * kPointStep bytes.
 */
inline void packPointCloud(const unilidar_sdk2::PointCloudUnitree & cloud, uint8_t * dst)
{
  for (const unilidar_sdk2::PointUnitree & point : cloud.points) {
    PackedPoint packed{};
    packed.x = point.x;
    packed.y = point.y;
    packed.z = point.z;
    packed.w = 1.0f;
    packed.intensity = point.intensity;
    packed.ring = static_cast<uint16_t>(point.ring);
    packed.time = point.time;

    // memcpy rather than a reinterpret_cast: the destination comes from a
    // std::vector<uint8_t> and is not guaranteed to be suitably aligned for a
    // 16 byte aligned type.
    std::memcpy(dst, &packed, sizeof(packed));
    dst += sizeof(packed);
  }
}

}  // namespace unitree_lidar_ros2

#endif  // UNITREE_LIDAR_ROS2__POINT_CLOUD_LAYOUT_HPP_
