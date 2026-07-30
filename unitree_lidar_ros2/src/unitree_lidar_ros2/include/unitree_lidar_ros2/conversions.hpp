/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef UNITREE_LIDAR_ROS2__CONVERSIONS_HPP_
#define UNITREE_LIDAR_ROS2__CONVERSIONS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "unitree_lidar_sdk.h"  // NOLINT(build/include_subdir)

/// Pure conversions shared by the driver, kept free of any ROS dependency so
/// they can be unit tested on their own.
namespace unitree_lidar_ros2
{

/// Smallest quaternion norm still considered usable.
constexpr double kMinQuaternionNorm = 1e-6;

/**
 * @brief Converts a Unix timestamp in seconds to nanoseconds.
 *
 * The whole and fractional parts are scaled separately on purpose. Evaluating
 * `seconds * 1e9` in one step throws away the low digits, because a present day
 * Unix timestamp already uses most of a double's 53 bit mantissa: the value is
 * around 1.7e9, so the remaining resolution is only about 1e-7 s, and scaling
 * afterwards cannot recover what was lost.
 *
 * @param seconds Seconds since the Unix epoch. Must be non-negative; the caller
 *        is expected to have rejected sensor stamps that are zero or negative.
 * @return The same instant in nanoseconds.
 */
inline int64_t sensorStampToNanoseconds(double seconds)
{
  const double whole_seconds = std::floor(seconds);
  const double fraction = seconds - whole_seconds;
  // llround, not a cast: a fraction of 0.9999999999 has to become the next whole
  // second rather than being truncated back down.
  return static_cast<int64_t>(whole_seconds) * 1000000000LL +
         static_cast<int64_t>(std::llround(fraction * 1e9));
}

/// A normalised orientation, in the order the ROS message uses.
struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/**
 * @brief Normalises the orientation carried by an SDK IMU sample.
 *
 * The SDK stores the quaternion as (x, y, z, w) - the order its own example
 * prints - and the array is copied straight across without reordering.
 *
 * @param quaternion The SDK's four element array.
 * @param[out] out The normalised orientation, untouched when this returns false.
 * @return false when the quaternion cannot be normalised, which is what an all
 *         zero array looks like when the lidar has its IMU disabled. tf2 and most
 *         consumers reject such a value, so the sample is better dropped than
 *         published.
 */
inline bool normalizeOrientation(const float quaternion[4], Orientation & out)
{
  const double x = quaternion[0];
  const double y = quaternion[1];
  const double z = quaternion[2];
  const double w = quaternion[3];

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < kMinQuaternionNorm) {
    return false;
  }

  out.x = x / norm;
  out.y = y / norm;
  out.z = z / norm;
  out.w = w / norm;
  return true;
}

/**
 * @brief Clamps a count taken off the wire to the capacity that will be indexed.
 *
 * point_num arrives inside the packet, so a corrupted or truncated packet would
 * otherwise walk off the end of the ranges array.
 */
inline size_t clampPointCount(uint32_t point_num, size_t capacity)
{
  return std::min<size_t>(point_num, capacity);
}

/// Range window applied to a 2D scan, in metres.
struct RangeWindow
{
  float min{0.0f};
  float max{0.0f};

  /// True when the window can contain anything at all.
  bool valid() const {return max > min;}
};

/**
 * @brief Narrows the configured range window with the limits the lidar reports.
 *
 * The packet's own limits win when they look sane; some firmware leaves them
 * empty, in which case the configured window is used unchanged.
 */
inline RangeWindow resolveRangeWindow(
  double configured_min, double configured_max, float packet_min, float packet_max)
{
  RangeWindow window{static_cast<float>(configured_min), static_cast<float>(configured_max)};
  if (packet_max > packet_min) {
    window.min = std::max(window.min, packet_min);
    window.max = std::min(window.max, packet_max);
  }
  return window;
}

/**
 * @brief Converts one raw 2D range reading to metres.
 *
 * @return The range in metres, or infinity for a reading that is invalid or
 *         outside @p window. LaserScan asks for discarded readings to be outside
 *         [range_min, range_max]; publishing 0.0 instead, as this driver used to,
 *         reads as a real return right at the sensor.
 */
inline float convertScanRange(
  uint16_t raw, const unilidar_sdk2::LidarCalibParam & param, const RangeWindow & window)
{
  const float invalid = std::numeric_limits<float>::infinity();
  if (raw < 1) {
    return invalid;
  }

  const float range = param.range_scale * (static_cast<float>(raw) + param.range_bias);
  if (range < window.min || range > window.max) {
    return invalid;
  }
  return range;
}

/**
 * @brief Angle of the last sample in a scan of @p num_points, in radians.
 *
 * @param angle_min Angle of the first sample, calibration bias already applied.
 */
inline float scanAngleMax(float angle_min, float angle_increment, size_t num_points)
{
  if (num_points == 0) {
    return angle_min;
  }
  return angle_min + angle_increment * static_cast<float>(num_points - 1);
}

}  // namespace unitree_lidar_ros2

#endif  // UNITREE_LIDAR_ROS2__CONVERSIONS_HPP_
