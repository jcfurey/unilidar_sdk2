/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef PACKET_BUILDER_HPP_
#define PACKET_BUILDER_HPP_

// Test support: builds lidar packets the SDK's parser accepts, so the driver can
// be exercised without hardware.
//
// Note the frame checksum covers the *payload only*, despite the comment on
// FrameTail::crc32 in unitree_lidar_protocol.h saying "crc check of head and
// data". That was established empirically: a checksum over header + payload is
// rejected, one over the payload alone is accepted.

#include <arpa/inet.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "unitree_lidar_sdk.h"  // NOLINT(build/include_subdir)
#include "unitree_lidar_utilities.h"  // NOLINT(build/include_subdir)

namespace unitree_lidar_ros2
{
namespace test
{

/// Sends UDP datagrams to a driver listening on loopback.
class PacketSender
{
public:
  PacketSender(const std::string & ip, uint16_t port)
  {
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    std::memset(&destination_, 0, sizeof(destination_));
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(port);
    ::inet_pton(AF_INET, ip.c_str(), &destination_.sin_addr);
  }

  ~PacketSender()
  {
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  PacketSender(const PacketSender &) = delete;
  PacketSender & operator=(const PacketSender &) = delete;

  bool valid() const {return socket_ >= 0;}

  /// Stamps the frame header and tail of @p packet, then sends it.
  template<typename PacketT>
  bool send(PacketT & packet, uint32_t packet_type)
  {
    packet.header.header[0] = 0x55;
    packet.header.header[1] = 0xAA;
    packet.header.header[2] = 0x05;
    packet.header.header[3] = 0x0A;
    packet.header.packet_type = packet_type;
    packet.header.packet_size = sizeof(PacketT);

    packet.data.info.payload_size = sizeof(packet.data);

    packet.tail.crc32 = unilidar_sdk2::crc32(
      reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));
    packet.tail.msg_type_check = packet_type;
    packet.tail.tail[0] = 0x00;
    packet.tail.tail[1] = 0xFF;

    const ssize_t sent = ::sendto(
      socket_, &packet, sizeof(PacketT), 0,
      reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
    return sent == static_cast<ssize_t>(sizeof(PacketT));
  }

private:
  int socket_{-1};
  sockaddr_in destination_{};
};

/// An IMU packet with a known, already normalised orientation.
inline unilidar_sdk2::LidarImuDataPacket makeImuPacket()
{
  unilidar_sdk2::LidarImuDataPacket packet{};
  packet.data.info.seq = 4242;
  packet.data.info.stamp.sec = 1730191291u;
  // Deliberately small, to catch a nanosecond field that is not zero padded.
  packet.data.info.stamp.nsec = 4411172u;

  // (x, y, z, w) with norm 1.
  packet.data.quaternion[0] = 0.1f;
  packet.data.quaternion[1] = 0.2f;
  packet.data.quaternion[2] = 0.3f;
  packet.data.quaternion[3] = 0.9273618f;

  packet.data.angular_velocity[0] = 1.5f;
  packet.data.angular_velocity[1] = -2.5f;
  packet.data.angular_velocity[2] = 0.25f;

  packet.data.linear_acceleration[0] = 0.11f;
  packet.data.linear_acceleration[1] = 0.22f;
  packet.data.linear_acceleration[2] = 9.81f;
  return packet;
}

/// A 3D scan line holding @p num_points returns of 1 m, 2 m, ... in range.
inline unilidar_sdk2::LidarPointDataPacket makePointPacket(uint32_t num_points)
{
  unilidar_sdk2::LidarPointDataPacket packet{};
  packet.data.info.seq = 77;
  packet.data.info.stamp.sec = 1730191292u;
  packet.data.info.stamp.nsec = 500000000u;

  packet.data.point_num = num_points;
  packet.data.range_min = 0.1f;
  packet.data.range_max = 90.0f;
  packet.data.time_increment = 1e-5f;
  packet.data.scan_period = 0.1f;

  // Both sweeps are held at zero so every return lands on the +Y axis at exactly
  // (0, range, 0). The point of this fixture is the driver's plumbing - field
  // offsets, counts, frames - not the SDK's trigonometry, and a stationary beam
  // makes the expected values exact rather than approximate.
  packet.data.angle_min = 0.0f;
  packet.data.angle_increment = 0.0f;
  packet.data.com_horizontal_angle_start = 0.0f;
  packet.data.com_horizontal_angle_step = 0.0f;

  // Identity calibration: ranges are millimetres, no bias or axis offsets.
  packet.data.param.range_scale = 0.001f;
  packet.data.param.range_bias = 0.0f;

  const uint32_t capacity = sizeof(packet.data.ranges) / sizeof(packet.data.ranges[0]);
  const uint32_t filled = num_points < capacity ? num_points : capacity;
  for (uint32_t i = 0; i < filled; ++i) {
    packet.data.ranges[i] = static_cast<uint16_t>(1000 * (i + 1));
    packet.data.intensities[i] = static_cast<uint8_t>(10 * (i + 1));
  }
  return packet;
}

}  // namespace test
}  // namespace unitree_lidar_ros2

#endif  // PACKET_BUILDER_HPP_
