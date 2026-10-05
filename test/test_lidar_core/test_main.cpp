// Host-side tests for lib/LidarCore. Run with: pio test -e native

#include <CoinD6Decoder.h>
#include <PolarScan.h>
#include <Proximity.h>
#include <unity.h>

#include <cmath>
#include <vector>

using namespace lidarcore;

void setUp() {}
void tearDown() {}

// ---------- helpers ----------

struct Sample {
  float angle;
  uint16_t distance;
};

struct RecordingSink : PointSink {
  int revolutions = 0;
  std::vector<Sample> samples;
  void onRevolutionStart() override { ++revolutions; }
  void onSample(float angleDeg, uint16_t distanceMm) override {
    samples.push_back({angleDeg, distanceMm});
  }
};

uint16_t encodeAngle(float deg) { return static_cast<uint16_t>(std::lround(deg * 64.0f)) << 1 | 1; }

// Builds a valid COIN-D6 packet.
std::vector<uint8_t> makePacket(bool ringStart, float startDeg, float endDeg,
                                const std::vector<uint16_t>& distances) {
  const uint8_t ct = ringStart ? 1 : 0;
  const uint8_t lsn = static_cast<uint8_t>(distances.size());
  const uint16_t fsa = encodeAngle(startDeg);
  const uint16_t lsa = encodeAngle(endDeg);

  std::vector<uint8_t> samples;
  uint16_t cs = 0x55AA ^ static_cast<uint16_t>(ct | lsn << 8) ^ fsa ^ lsa;
  for (uint16_t d : distances) {
    const uint8_t sL = 0x10;                                       // intensity bits
    const uint8_t s2 = static_cast<uint8_t>((d & 0x3F) << 2);      // low 6 bits of distance
    const uint8_t sH = static_cast<uint8_t>(d >> 6);
    samples.insert(samples.end(), {sL, s2, sH});
    cs ^= sL;
    cs ^= static_cast<uint16_t>(sH << 8 | s2);
  }

  std::vector<uint8_t> p = {0xAA, 0x55, ct, lsn,
                            static_cast<uint8_t>(fsa), static_cast<uint8_t>(fsa >> 8),
                            static_cast<uint8_t>(lsa), static_cast<uint8_t>(lsa >> 8),
                            static_cast<uint8_t>(cs), static_cast<uint8_t>(cs >> 8)};
  p.insert(p.end(), samples.begin(), samples.end());
  return p;
}

void feedAll(CoinD6Decoder& decoder, const std::vector<uint8_t>& bytes) {
  decoder.feed(bytes.data(), bytes.size());
}

// ---------- decoder ----------

void test_decodes_distances_and_interpolates_angles() {
  RecordingSink sink;
  CoinD6Decoder decoder(sink);
  feedAll(decoder, makePacket(false, 10.0f, 13.0f, {500, 1000, 1500, 12000}));

  TEST_ASSERT_EQUAL_UINT32(1, decoder.validPackets());
  TEST_ASSERT_EQUAL(4, sink.samples.size());
  TEST_ASSERT_EQUAL_UINT16(500, sink.samples[0].distance);
  TEST_ASSERT_EQUAL_UINT16(12000, sink.samples[3].distance);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 10.0f, sink.samples[0].angle);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 11.0f, sink.samples[1].angle);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 13.0f, sink.samples[3].angle);
  TEST_ASSERT_EQUAL(0, sink.revolutions);
}

void test_wraps_angles_across_zero() {
  RecordingSink sink;
  CoinD6Decoder decoder(sink);
  feedAll(decoder, makePacket(false, 359.0f, 1.0f, {100, 100, 100}));
  TEST_ASSERT_EQUAL(3, sink.samples.size());
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 359.0f, sink.samples[0].angle);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, sink.samples[1].angle);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, sink.samples[2].angle);
}

void test_reports_ring_start() {
  RecordingSink sink;
  CoinD6Decoder decoder(sink);
  feedAll(decoder, makePacket(true, 0.0f, 0.0f, {200}));
  TEST_ASSERT_EQUAL(1, sink.revolutions);
}

void test_rejects_bad_checksum_and_resyncs() {
  RecordingSink sink;
  CoinD6Decoder decoder(sink);
  auto bad = makePacket(false, 0.0f, 1.0f, {300, 300});
  bad[10] ^= 0xFF;  // corrupt a sample byte
  const uint8_t noise[] = {0x00, 0xAA, 0x12, 0x55, 0xFF};
  decoder.feed(noise, sizeof(noise));
  feedAll(decoder, bad);
  feedAll(decoder, makePacket(false, 5.0f, 6.0f, {700, 800}));

  TEST_ASSERT_EQUAL_UINT32(1, decoder.checksumErrors());
  TEST_ASSERT_EQUAL_UINT32(1, decoder.validPackets());
  TEST_ASSERT_EQUAL(2, sink.samples.size());
  TEST_ASSERT_EQUAL_UINT16(700, sink.samples[0].distance);
}

void test_handles_byte_by_byte_feeding() {
  RecordingSink sink;
  CoinD6Decoder decoder(sink);
  for (uint8_t b : makePacket(false, 90.0f, 91.0f, {1234, 4321})) decoder.feed(b);
  TEST_ASSERT_EQUAL(2, sink.samples.size());
  TEST_ASSERT_EQUAL_UINT16(4321, sink.samples[1].distance);
}

// ---------- scan builder ----------

ScanFilter testFilter() { return {92.5f, 110.0f, 20.0f, 12000, 3}; }

void test_builder_publishes_only_complete_revolutions() {
  ScanBuilder builder({0.0f, true, 0.0f, 0.0f}, testFilter());
  PolarScan scan;

  builder.onSample(0.0f, 1000);  // before the first ring start: ignored
  builder.onRevolutionStart();
  TEST_ASSERT_FALSE(builder.takeScan(scan));
  builder.onSample(0.2f, 1000);
  builder.onSample(90.5f, 2000);
  builder.onSample(90.7f, 1500);  // same bin, nearer -> kept
  builder.onRevolutionStart();

  TEST_ASSERT_TRUE(builder.takeScan(scan));
  TEST_ASSERT_EQUAL_UINT16(1000, scan.distanceMm[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, scan.distanceMm[90]);
  TEST_ASSERT_EQUAL_UINT16(0, scan.distanceMm[180]);
  TEST_ASSERT_FALSE(builder.takeScan(scan));  // already taken
}

void test_builder_discards_short_revolutions() {
  ScanBuilder builder({0.0f, true, 0.0f, 0.0f}, testFilter());
  PolarScan scan;
  builder.onRevolutionStart();
  builder.onSample(10.0f, 1000);  // only 1 of the 3 required samples
  builder.onRevolutionStart();
  TEST_ASSERT_FALSE(builder.takeScan(scan));
}

void test_builder_applies_backwards_mount() {
  // Sensor 0° points backwards: what the sensor sees at 0° is behind the vehicle.
  ScanBuilder builder({180.0f, true, 0.0f, 0.0f}, testFilter());
  PolarScan scan;
  builder.onRevolutionStart();
  builder.onSample(0.5f, 1000);   // sensor front -> vehicle rear (180°)
  builder.onSample(90.5f, 1000);  // sensor right -> vehicle left (270°)
  builder.onSample(180.5f, 1000); // sensor rear  -> vehicle front (0°)
  builder.onRevolutionStart();
  TEST_ASSERT_TRUE(builder.takeScan(scan));
  TEST_ASSERT_EQUAL_UINT16(1000, scan.distanceMm[180]);
  TEST_ASSERT_EQUAL_UINT16(1000, scan.distanceMm[270]);
  TEST_ASSERT_EQUAL_UINT16(1000, scan.distanceMm[0]);
}

void test_builder_counter_clockwise_sensor_is_mirrored() {
  ScanBuilder builder({0.0f, false, 0.0f, 0.0f}, testFilter());
  PolarScan scan;
  builder.onRevolutionStart();
  builder.onSample(90.5f, 1000);  // counter-clockwise 90° = left
  builder.onSample(1.0f, 1000);
  builder.onSample(2.0f, 1000);
  builder.onRevolutionStart();
  TEST_ASSERT_TRUE(builder.takeScan(scan));
  TEST_ASSERT_EQUAL_UINT16(1000, scan.distanceMm[269]);
}

void test_builder_ignores_vehicle_body() {
  ScanBuilder builder({0.0f, true, 0.0f, 0.0f}, testFilter());
  PolarScan scan;
  builder.onRevolutionStart();
  builder.onSample(0.5f, 120);    // 12 cm ahead: inside length/2 + margin (130 mm)
  builder.onSample(90.5f, 100);   // 10 cm right: inside width/2 + margin
  builder.onSample(0.5f, 0);      // no return
  builder.onRevolutionStart();
  TEST_ASSERT_TRUE(builder.takeScan(scan));
  TEST_ASSERT_EQUAL_UINT16(0, scan.distanceMm[0]);
  TEST_ASSERT_EQUAL_UINT16(0, scan.distanceMm[90]);
  TEST_ASSERT_EQUAL_UINT16(0, scan.pointCount);
}

void test_builder_applies_mount_offset() {
  // Sensor 100 mm in front of the centre: a wall 500 mm ahead of the sensor is
  // 600 mm ahead of the vehicle centre.
  ScanBuilder builder({0.0f, true, 0.0f, 100.0f}, testFilter());
  PolarScan scan;
  builder.onRevolutionStart();
  builder.onSample(0.5f, 500);
  builder.onSample(10.0f, 0);
  builder.onSample(20.0f, 0);
  builder.onRevolutionStart();
  TEST_ASSERT_TRUE(builder.takeScan(scan));
  TEST_ASSERT_UINT16_WITHIN(1, 600, scan.distanceMm[0]);
}

// ---------- proximity ----------

void test_clearance_ahead_and_behind() {
  PolarScan scan;
  scan.distanceMm[0] = 500;     // 500 mm ahead of the centre
  scan.distanceMm[180] = 1000;  // 1000 mm behind
  scan.distanceMm[90] = 300;    // to the right, outside the corridor
  const float front = corridorClearance(scan, true, 92.5f, 110.0f, 30.0f, 12000.0f);
  const float rear = corridorClearance(scan, false, 92.5f, 110.0f, 30.0f, 12000.0f);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 390.0f, front);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 890.0f, rear);
}

void test_clearance_empty_corridor() {
  PolarScan scan;
  scan.distanceMm[45] = 400;  // diagonal, x ~ 283 mm: outside a 122.5 mm corridor
  TEST_ASSERT_EQUAL_FLOAT(12000.0f, corridorClearance(scan, true, 92.5f, 110.0f, 30.0f, 12000.0f));
}

void test_clearance_sees_obstacle_at_corridor_edge() {
  PolarScan scan;
  // ~110 mm to the right, ~600 mm ahead: inside the 122.5 mm half corridor.
  scan.distanceMm[10] = 610;
  const float front = corridorClearance(scan, true, 92.5f, 110.0f, 30.0f, 12000.0f);
  TEST_ASSERT_TRUE(front < 600.0f);
}

void test_speed_factor() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, speedFactorForClearance(100.0f, 150.0f, 600.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, speedFactorForClearance(150.0f, 150.0f, 600.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, speedFactorForClearance(375.0f, 150.0f, 600.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedFactorForClearance(5000.0f, 150.0f, 600.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_decodes_distances_and_interpolates_angles);
  RUN_TEST(test_wraps_angles_across_zero);
  RUN_TEST(test_reports_ring_start);
  RUN_TEST(test_rejects_bad_checksum_and_resyncs);
  RUN_TEST(test_handles_byte_by_byte_feeding);
  RUN_TEST(test_builder_publishes_only_complete_revolutions);
  RUN_TEST(test_builder_discards_short_revolutions);
  RUN_TEST(test_builder_applies_backwards_mount);
  RUN_TEST(test_builder_counter_clockwise_sensor_is_mirrored);
  RUN_TEST(test_builder_ignores_vehicle_body);
  RUN_TEST(test_builder_applies_mount_offset);
  RUN_TEST(test_clearance_ahead_and_behind);
  RUN_TEST(test_clearance_empty_corridor);
  RUN_TEST(test_clearance_sees_obstacle_at_corridor_edge);
  RUN_TEST(test_speed_factor);
  return UNITY_END();
}
