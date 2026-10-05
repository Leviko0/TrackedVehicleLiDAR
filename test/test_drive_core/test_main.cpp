// Host-side tests for lib/DriveCore. Run with: pio test -e native

#include <CommandParser.h>
#include <DriveMath.h>
#include <unity.h>

using namespace drivecore;

void setUp() {}
void tearDown() {}

// ---------- mixing ----------

void test_mix_forward_drives_both_tracks_forward() {
  TrackSpeeds s = mixArcade(1.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.left);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.right);
}

void test_mix_backward_drives_both_tracks_backward() {
  TrackSpeeds s = mixArcade(-1.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, s.left);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, s.right);
}

void test_mix_clockwise_spins_on_the_spot() {
  TrackSpeeds s = mixArcade(0.0f, 1.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.left);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, s.right);
}

void test_mix_counter_clockwise_spins_on_the_spot() {
  TrackSpeeds s = mixArcade(0.0f, -1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, s.left);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.right);
}

void test_mix_scales_down_keeping_ratio() {
  TrackSpeeds s = mixArcade(1.0f, 0.5f);  // raw: 1.5 / 0.5
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f / 3.0f, s.right);
}

void test_mix_clamps_out_of_range_input() {
  TrackSpeeds s = mixArcade(5.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.left);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.right);
}

// ---------- helpers ----------

void test_deadband_zeroes_small_values_and_rescales() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyDeadband(0.04f, 0.05f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyDeadband(-0.05f, 0.05f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, applyDeadband(1.0f, 0.05f));
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, applyDeadband(-1.0f, 0.05f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, applyDeadband(0.525f, 0.05f));
}

void test_slew_limits_step_size() {
  TEST_ASSERT_EQUAL_FLOAT(0.1f, slewToward(0.0f, 1.0f, 0.1f));
  TEST_ASSERT_EQUAL_FLOAT(-0.1f, slewToward(0.0f, -1.0f, 0.1f));
  TEST_ASSERT_EQUAL_FLOAT(0.55f, slewToward(0.5f, 0.55f, 0.1f));
}

void test_ramp_accelerates_slowly_and_brakes_fast() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, rampToward(0.0f, 1.0f, 0.1f, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, rampToward(0.8f, 0.0f, 0.1f, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5f, rampToward(-0.8f, 0.0f, 0.1f, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, rampToward(0.5f, 0.4f, 0.1f, 0.3f));
}

void test_ramp_brakes_to_zero_before_reversing() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rampToward(0.2f, -1.0f, 0.1f, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.1f, rampToward(0.0f, -1.0f, 0.1f, 0.3f));
}

void test_speed_to_duty_respects_min_duty() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, speedToDuty(0.0f, 0.25f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedToDuty(1.0f, 0.25f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.625f, speedToDuty(0.5f, 0.25f));
}

void test_clamp_turns_nan_into_zero() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, clampUnit(0.0f / 0.0f));
}

// ---------- protocol ----------

void test_parse_drive() {
  Command c = parseCommand("D 0.50 -0.25");
  TEST_ASSERT_TRUE(c.type == CommandType::Drive);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, c.throttle);
  TEST_ASSERT_EQUAL_FLOAT(-0.25f, c.turn);
}

void test_parse_drive_clamps_values() {
  Command c = parseCommand("D 3 -7");
  TEST_ASSERT_TRUE(c.type == CommandType::Drive);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, c.throttle);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, c.turn);
}

void test_parse_speed_limit() {
  Command c = parseCommand("L 0.8");
  TEST_ASSERT_TRUE(c.type == CommandType::SpeedLimit);
  TEST_ASSERT_EQUAL_FLOAT(0.8f, c.value);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, parseCommand("L 2").value);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parseCommand("L -1").value);
}

void test_parse_guard() {
  Command on = parseCommand("G 1");
  TEST_ASSERT_TRUE(on.type == CommandType::Guard);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, on.value);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parseCommand("G 0").value);
  TEST_ASSERT_TRUE(parseCommand("G 0").type == CommandType::Guard);
  TEST_ASSERT_TRUE(parseCommand("G 2").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("G").type == CommandType::Invalid);
}

void test_parse_stop_and_ping() {
  TEST_ASSERT_TRUE(parseCommand("S").type == CommandType::Stop);
  TEST_ASSERT_TRUE(parseCommand("P\n").type == CommandType::Ping);
}

void test_parse_rejects_garbage() {
  TEST_ASSERT_TRUE(parseCommand(nullptr).type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("X 1 2").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("D 1").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("D 1 2 3").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("D a b").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("D nan 0").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("S now").type == CommandType::Invalid);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_mix_forward_drives_both_tracks_forward);
  RUN_TEST(test_mix_backward_drives_both_tracks_backward);
  RUN_TEST(test_mix_clockwise_spins_on_the_spot);
  RUN_TEST(test_mix_counter_clockwise_spins_on_the_spot);
  RUN_TEST(test_mix_scales_down_keeping_ratio);
  RUN_TEST(test_mix_clamps_out_of_range_input);
  RUN_TEST(test_deadband_zeroes_small_values_and_rescales);
  RUN_TEST(test_slew_limits_step_size);
  RUN_TEST(test_ramp_accelerates_slowly_and_brakes_fast);
  RUN_TEST(test_ramp_brakes_to_zero_before_reversing);
  RUN_TEST(test_speed_to_duty_respects_min_duty);
  RUN_TEST(test_clamp_turns_nan_into_zero);
  RUN_TEST(test_parse_drive);
  RUN_TEST(test_parse_drive_clamps_values);
  RUN_TEST(test_parse_speed_limit);
  RUN_TEST(test_parse_guard);
  RUN_TEST(test_parse_stop_and_ping);
  RUN_TEST(test_parse_rejects_garbage);
  return UNITY_END();
}
