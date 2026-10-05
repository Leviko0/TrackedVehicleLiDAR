#include "CoinD6Decoder.h"

namespace lidarcore {

namespace {
constexpr uint8_t HEADER_0 = 0xAA;
constexpr uint8_t HEADER_1 = 0x55;

uint16_t readU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

float rawToDegrees(uint16_t raw) { return static_cast<float>(raw >> 1) / 64.0f; }
}  // namespace

constexpr uint8_t CoinD6Decoder::START_COMMAND[4];
constexpr uint8_t CoinD6Decoder::STOP_COMMAND[4];

void CoinD6Decoder::reset() {
  length_ = 0;
  sawHeaderByte0_ = false;
}

void CoinD6Decoder::feed(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) feed(data[i]);
}

void CoinD6Decoder::feed(uint8_t byte) {
  // Searching for the 0xAA 0x55 header.
  if (length_ == 0) {
    if (sawHeaderByte0_ && byte == HEADER_1) {
      buffer_[0] = HEADER_0;
      buffer_[1] = HEADER_1;
      length_ = 2;
      sawHeaderByte0_ = false;
    } else {
      sawHeaderByte0_ = (byte == HEADER_0);
    }
    return;
  }

  buffer_[length_++] = byte;
  if (length_ < PREAMBLE_LENGTH) return;

  const size_t expected = PREAMBLE_LENGTH + 3 * static_cast<size_t>(buffer_[3]);
  if (length_ < expected) return;

  finishPacket();
  reset();
}

void CoinD6Decoder::finishPacket() {
  const uint8_t sampleCount = buffer_[3];

  uint16_t checksum = 0x55AA;
  checksum ^= static_cast<uint16_t>(buffer_[2] | (buffer_[3] << 8));
  checksum ^= readU16(&buffer_[4]);
  checksum ^= readU16(&buffer_[6]);
  for (size_t i = 0; i < sampleCount; ++i) {
    const uint8_t* s = &buffer_[PREAMBLE_LENGTH + 3 * i];
    checksum ^= s[0];
    checksum ^= static_cast<uint16_t>((s[2] << 8) | s[1]);
  }
  if (checksum != readU16(&buffer_[8])) {
    ++checksumErrors_;
    return;
  }
  ++validPackets_;

  if (buffer_[2] & 0x01) sink_.onRevolutionStart();
  if (sampleCount == 0) return;

  const float startDeg = rawToDegrees(readU16(&buffer_[4]));
  float endDeg = rawToDegrees(readU16(&buffer_[6]));
  if (endDeg < startDeg) endDeg += 360.0f;  // packet crosses 0°
  const float step = sampleCount > 1 ? (endDeg - startDeg) / (sampleCount - 1) : 0.0f;

  for (size_t i = 0; i < sampleCount; ++i) {
    const uint8_t* s = &buffer_[PREAMBLE_LENGTH + 3 * i];
    const uint16_t distance = static_cast<uint16_t>(s[2] * 64 + (s[1] >> 2));
    float angle = startDeg + step * static_cast<float>(i);
    if (angle >= 360.0f) angle -= 360.0f;
    sink_.onSample(angle, distance);
  }
}

}  // namespace lidarcore
