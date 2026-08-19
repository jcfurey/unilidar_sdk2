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

/// Number of nanoseconds in one second, shared by the exact packet-stamp helpers.
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

/// The L2 firmware exposes the point-packet sequence as a uint32, but the
/// hardware counter itself is 10 bit and rolls from 1023 back to zero.
constexpr uint32_t kPointPacketSequenceModulus = 1024u;

/// True when a timestamp received from the lidar can be represented as ROS time.
inline bool validPacketTimestamp(const unilidar_sdk2::TimeStamp & stamp)
{
  return stamp.sec > 0u && stamp.nsec < static_cast<uint32_t>(kNanosecondsPerSecond);
}

/**
 * @brief Converts the lidar's integer seconds/nanoseconds timestamp without a
 *        precision-losing intermediate `double`.
 */
inline int64_t packetTimestampToNanoseconds(const unilidar_sdk2::TimeStamp & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nsec;
}

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

/// True when the metadata and calibration needed for a 3D scan line are usable.
inline bool validPointCloudMetadata(const unilidar_sdk2::LidarPointData & data)
{
  const auto & calibration = data.param;
  const bool packet_range_valid =
    (data.range_min == 0.0f && data.range_max == 0.0f) ||
    (std::isfinite(data.range_min) && std::isfinite(data.range_max) &&
    data.range_max > data.range_min);

  return std::isfinite(data.com_horizontal_angle_start) &&
         std::isfinite(data.com_horizontal_angle_step) &&
         std::isfinite(data.scan_period) && data.scan_period >= 0.0f &&
         packet_range_valid &&
         std::isfinite(data.angle_min) && std::isfinite(data.angle_increment) &&
         std::isfinite(data.time_increment) && data.time_increment >= 0.0f &&
         std::isfinite(calibration.a_axis_dist) &&
         std::isfinite(calibration.b_axis_dist) &&
         std::isfinite(calibration.theta_angle_bias) &&
         std::isfinite(calibration.alpha_angle_bias) &&
         std::isfinite(calibration.beta_angle) &&
         std::isfinite(calibration.xi_angle) &&
         std::isfinite(calibration.range_bias) &&
         std::isfinite(calibration.range_scale) && calibration.range_scale > 0.0f;
}

/// True when a converted SDK point is safe to advertise in a dense cloud.
inline bool validPoint(const unilidar_sdk2::PointUnitree & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(point.intensity) && std::isfinite(point.time) && point.time >= 0.0f;
}

/// Classifies a sequence mismatch. A small unsigned delta is forward loss; a
/// large one is a duplicate or out-of-order packet. Wrap from UINT32_MAX to zero
/// naturally has a delta of zero.
inline uint32_t missingPacketCount(
  uint32_t expected, uint32_t actual, bool & out_of_order, uint32_t modulus = 0u)
{
  const uint32_t delta = modulus == 0u ?
    actual - expected : (actual + modulus - expected) % modulus;
  const uint32_t backward_threshold = modulus == 0u ? 0x80000000u : modulus / 2u;
  out_of_order = delta >= backward_threshold;
  return out_of_order ? 0u : delta;
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

/// True when the metadata needed to construct a valid LaserScan is finite and
/// the raw-to-metre scale is usable.
inline bool validLaserScanMetadata(const unilidar_sdk2::Lidar2DPointData & data)
{
  return std::isfinite(data.angle_min) && std::isfinite(data.angle_increment) &&
         std::isfinite(data.time_increment) && data.time_increment >= 0.0f &&
         std::isfinite(data.scan_period) && data.scan_period >= 0.0f &&
         std::isfinite(data.param.alpha_angle_bias) &&
         std::isfinite(data.param.range_scale) && data.param.range_scale > 0.0f &&
         std::isfinite(data.param.range_bias);
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
  if (!std::isfinite(range) || range < window.min || range > window.max) {
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
