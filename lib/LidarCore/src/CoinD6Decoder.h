#pragma once

#include <cstddef>
#include <cstdint>

// Byte-stream decoder for the COIN-D6 360° dTOF LiDAR (230400 baud, 8N1).
//
// Packet layout (little endian):
//   0-1  header        0xAA 0x55
//   2    CT            bit 0 = 1 -> first packet of a new revolution
//   3    LSN           number of samples in this packet
//   4-5  FSA           start angle, degrees = (FSA >> 1) / 64
//   6-7  LSA           end angle,   degrees = (LSA >> 1) / 64
//   8-9  CS            16-bit XOR checksum
//   10.. samples       3 bytes each: Si_L, Si_2nd, Si_H
//                      distance mm = Si_H * 64 + (Si_2nd >> 2)   (0 = no return)
//
// Checksum = 0x55AA ^ (CT | LSN << 8) ^ FSA ^ LSA ^ for each sample
//            (Si_L ^ (Si_H << 8 | Si_2nd)).

namespace lidarcore {

// Receives decoded samples. Angles are in the sensor's own frame.
class PointSink {
 public:
  virtual ~PointSink() = default;
  // Called before the samples of a packet that starts a new revolution.
  virtual void onRevolutionStart() = 0;
  // distanceMm == 0 means the sensor got no return at that angle.
  virtual void onSample(float angleDeg, uint16_t distanceMm) = 0;
};

class CoinD6Decoder {
 public:
  // Start / stop scanning commands to send to the sensor's RX line.
  static constexpr uint8_t START_COMMAND[4] = {0xAA, 0x55, 0xF0, 0x0F};
  static constexpr uint8_t STOP_COMMAND[4] = {0xAA, 0x55, 0xF5, 0x0A};

  explicit CoinD6Decoder(PointSink& sink) : sink_(sink) {}

  void feed(uint8_t byte);
  void feed(const uint8_t* data, size_t length);

  // Drop any partial packet, e.g. after a UART overflow.
  void reset();

  uint32_t validPackets() const { return validPackets_; }
  uint32_t checksumErrors() const { return checksumErrors_; }

 private:
  static constexpr size_t PREAMBLE_LENGTH = 10;
  static constexpr size_t MAX_PACKET_LENGTH = PREAMBLE_LENGTH + 3 * 255;

  void finishPacket();

  PointSink& sink_;
  uint8_t buffer_[MAX_PACKET_LENGTH];
  size_t length_ = 0;
  bool sawHeaderByte0_ = false;
  uint32_t validPackets_ = 0;
  uint32_t checksumErrors_ = 0;
};

}  // namespace lidarcore
