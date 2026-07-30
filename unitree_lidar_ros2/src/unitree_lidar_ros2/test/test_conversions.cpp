/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "unitree_lidar_ros2/conversions.hpp"

namespace
{
using unitree_lidar_ros2::Orientation;
using unitree_lidar_ros2::RangeWindow;
using unitree_lidar_ros2::clampPointCount;
using unitree_lidar_ros2::convertScanRange;
using unitree_lidar_ros2::normalizeOrientation;
using unitree_lidar_ros2::resolveRangeWindow;
using unitree_lidar_ros2::scanAngleMax;
using unitree_lidar_ros2::sensorStampToNanoseconds;

constexpr float kInfinity = std::numeric_limits<float>::infinity();
}  // namespace

// ---------------------------------------------------------------------------
// Timestamps
// ---------------------------------------------------------------------------

TEST(SensorStamp, ConvertsWholeAndFractionalSeconds)
{
  EXPECT_EQ(sensorStampToNanoseconds(0.0), 0);
  EXPECT_EQ(sensorStampToNanoseconds(1.0), 1000000000LL);
  EXPECT_EQ(sensorStampToNanoseconds(1.5), 1500000000LL);
  EXPECT_EQ(sensorStampToNanoseconds(0.000000001), 1LL);
}

// The whole point of splitting the value before scaling: at Unix epoch magnitudes
// a double has only ~1e-7 s of resolution left, so `seconds * 1e9` in one step
// loses the tail. Splitting keeps the seconds exact and the sub-second part as
// good as the input.
TEST(SensorStamp, KeepsSecondsExactAtUnixEpochMagnitudes)
{
  const double stamp = 1730191291.25;
  const int64_t nanoseconds = sensorStampToNanoseconds(stamp);

  EXPECT_EQ(nanoseconds / 1000000000LL, 1730191291LL);
  EXPECT_EQ(nanoseconds % 1000000000LL, 250000000LL);

  // A naive single multiplication cannot even represent this exactly.
  const int64_t naive = static_cast<int64_t>(stamp * 1e9);
  EXPECT_NE(naive, nanoseconds);
}

TEST(SensorStamp, RoundsRatherThanTruncatesTheFraction)
{
  // 0.9999999999 s must become the next whole second, not 999999999 ns.
  EXPECT_EQ(sensorStampToNanoseconds(1.9999999999), 2000000000LL);
  // Nearest nanosecond, not the one below it.
  EXPECT_EQ(sensorStampToNanoseconds(2.0000000006), 2000000001LL);
}

TEST(SensorStamp, IsMonotonicAcrossASecondBoundary)
{
  EXPECT_LT(sensorStampToNanoseconds(1730191291.999), sensorStampToNanoseconds(1730191292.001));
}

// ---------------------------------------------------------------------------
// IMU orientation
// ---------------------------------------------------------------------------

TEST(Orientation, ReadsTheSdkArrayAsXyzw)
{
  // The SDK stores (x, y, z, w); its own example prints it in that order. The
  // transform broadcast used to read the same array as (w, x, y, z).
  const float quaternion[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  Orientation orientation;
  ASSERT_TRUE(normalizeOrientation(quaternion, orientation));
  EXPECT_DOUBLE_EQ(orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(orientation.w, 1.0);
}

TEST(Orientation, NormalisesANonUnitQuaternion)
{
  const float quaternion[4] = {0.0f, 0.0f, 0.0f, 4.0f};
  Orientation orientation;
  ASSERT_TRUE(normalizeOrientation(quaternion, orientation));
  EXPECT_DOUBLE_EQ(orientation.w, 1.0);

  const float mixed[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  ASSERT_TRUE(normalizeOrientation(mixed, orientation));
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-12);
  EXPECT_NEAR(orientation.x, 0.5, 1e-12);
}

// An all zero quaternion is what the lidar reports when its IMU is disabled by
// work_mode bit 2. tf2 rejects it, so the sample has to be dropped.
TEST(Orientation, RejectsADegenerateQuaternion)
{
  Orientation orientation;
  const float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  EXPECT_FALSE(normalizeOrientation(zeros, orientation));

  const float tiny[4] = {1e-9f, 0.0f, 0.0f, 0.0f};
  EXPECT_FALSE(normalizeOrientation(tiny, orientation));
}

TEST(Orientation, RejectsNonFiniteValues)
{
  Orientation orientation;
  const float nans[4] = {std::nanf(""), 0.0f, 0.0f, 1.0f};
  EXPECT_FALSE(normalizeOrientation(nans, orientation));

  const float infinities[4] = {kInfinity, 0.0f, 0.0f, 1.0f};
  EXPECT_FALSE(normalizeOrientation(infinities, orientation));
}

TEST(Orientation, LeavesTheOutputAloneWhenItFails)
{
  Orientation orientation;
  orientation.x = 0.25;
  const float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  ASSERT_FALSE(normalizeOrientation(zeros, orientation));
  EXPECT_DOUBLE_EQ(orientation.x, 0.25);
}

// ---------------------------------------------------------------------------
// Point counts taken off the wire
// ---------------------------------------------------------------------------

TEST(ClampPointCount, ClampsToTheIndexedCapacity)
{
  EXPECT_EQ(clampPointCount(0, 1800), 0u);
  EXPECT_EQ(clampPointCount(5, 1800), 5u);
  EXPECT_EQ(clampPointCount(1800, 1800), 1800u);
  // A corrupted packet must not be able to walk off the end of the array.
  EXPECT_EQ(clampPointCount(1801, 1800), 1800u);
  EXPECT_EQ(clampPointCount(0xFFFFFFFFu, 1800), 1800u);
  EXPECT_EQ(clampPointCount(0xFFFFFFFFu, 300), 300u);
}

// ---------------------------------------------------------------------------
// 2D scan ranges
// ---------------------------------------------------------------------------

TEST(ScanRange, AppliesScaleAndBias)
{
  unilidar_sdk2::LidarCalibParam param{};
  param.range_scale = 0.001f;
  param.range_bias = 0.0f;
  const RangeWindow window{0.0f, 100.0f};

  EXPECT_FLOAT_EQ(convertScanRange(1000, param, window), 1.0f);
  EXPECT_FLOAT_EQ(convertScanRange(2500, param, window), 2.5f);

  param.range_bias = 500.0f;
  EXPECT_FLOAT_EQ(convertScanRange(1000, param, window), 1.5f);
}

// LaserScan wants discarded readings outside [range_min, range_max]. Publishing
// 0.0, as this driver used to, reads as a real return right at the sensor.
TEST(ScanRange, MarksInvalidReturnsAsInfinite)
{
  unilidar_sdk2::LidarCalibParam param{};
  param.range_scale = 0.001f;
  const RangeWindow window{0.5f, 10.0f};

  EXPECT_EQ(convertScanRange(0, param, window), kInfinity) << "a raw 0 means no return";
  EXPECT_EQ(convertScanRange(100, param, window), kInfinity) << "0.1 m is below range_min";
  EXPECT_EQ(convertScanRange(60000, param, window), kInfinity) << "60 m is above range_max";
  EXPECT_FLOAT_EQ(convertScanRange(5000, param, window), 5.0f);
}

TEST(ScanRange, TreatsTheWindowAsInclusive)
{
  unilidar_sdk2::LidarCalibParam param{};
  param.range_scale = 0.001f;
  const RangeWindow window{1.0f, 5.0f};
  EXPECT_FLOAT_EQ(convertScanRange(1000, param, window), 1.0f);
  EXPECT_FLOAT_EQ(convertScanRange(5000, param, window), 5.0f);
  EXPECT_EQ(convertScanRange(999, param, window), kInfinity);
  EXPECT_EQ(convertScanRange(5001, param, window), kInfinity);
}

TEST(RangeWindowResolution, NarrowsTheConfiguredWindowWithThePacket)
{
  const RangeWindow window = resolveRangeWindow(0.0, 100.0, 0.2f, 30.0f);
  EXPECT_FLOAT_EQ(window.min, 0.2f);
  EXPECT_FLOAT_EQ(window.max, 30.0f);
  EXPECT_TRUE(window.valid());
}

TEST(RangeWindowResolution, KeepsTheTighterOfTheTwo)
{
  const RangeWindow window = resolveRangeWindow(1.0, 10.0, 0.2f, 30.0f);
  EXPECT_FLOAT_EQ(window.min, 1.0f);
  EXPECT_FLOAT_EQ(window.max, 10.0f);
}

// Some firmware leaves the packet limits empty; the configured window then stands.
TEST(RangeWindowResolution, IgnoresAnEmptyPacketWindow)
{
  const RangeWindow window = resolveRangeWindow(0.5, 40.0, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(window.min, 0.5f);
  EXPECT_FLOAT_EQ(window.max, 40.0f);
  EXPECT_TRUE(window.valid());
}

TEST(RangeWindowResolution, ReportsAnUnusableWindow)
{
  EXPECT_FALSE(resolveRangeWindow(10.0, 1.0, 0.0f, 0.0f).valid());
  EXPECT_FALSE(resolveRangeWindow(5.0, 5.0, 0.0f, 0.0f).valid());
}

// ---------------------------------------------------------------------------
// Scan angles
// ---------------------------------------------------------------------------

TEST(ScanAngleMax, CoversTheLastSampleNotOnePastIt)
{
  // 4 samples 0.5 rad apart starting at 0 end at 1.5, not 2.0.
  EXPECT_FLOAT_EQ(scanAngleMax(0.0f, 0.5f, 4), 1.5f);
  EXPECT_FLOAT_EQ(scanAngleMax(-1.0f, 0.25f, 5), 0.0f);
}

TEST(ScanAngleMax, HandlesDegenerateCounts)
{
  EXPECT_FLOAT_EQ(scanAngleMax(0.75f, 0.1f, 1), 0.75f);
  EXPECT_FLOAT_EQ(scanAngleMax(0.75f, 0.1f, 0), 0.75f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
