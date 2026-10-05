#pragma once

#include <cstddef>
#include <cstdint>

// Optional, git-ignored file holding your home Wi-Fi credentials.
// Copy include/Secrets.example.h to include/Secrets.h and fill it in.
#if __has_include("Secrets.h")
#include "Secrets.h"
#endif

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif

// Central place for every tunable value of the vehicle.
// Pin numbers follow docs/PinTable.md.
namespace config {

namespace pins {
// L298N channel A -> Motor A (left track)
constexpr uint8_t LEFT_ENABLE = 9;   // ENA (PWM)
constexpr uint8_t LEFT_IN1 = 10;     // IN1
constexpr uint8_t LEFT_IN2 = 11;     // IN2

// L298N channel B -> Motor B (right track)
constexpr uint8_t RIGHT_IN1 = 12;    // IN3
constexpr uint8_t RIGHT_IN2 = 13;    // IN4
constexpr uint8_t RIGHT_ENABLE = 14; // ENB (PWM)

// COIN-D6 LiDAR UART
constexpr uint8_t LIDAR_RX = 3;      // ESP RX  <- LiDAR TX
constexpr uint8_t LIDAR_TX = 1;      // ESP TX  -> LiDAR RX
}  // namespace pins

namespace motor {
constexpr uint32_t PWM_FREQUENCY_HZ = 10000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

// Smallest duty cycle that still makes the track move. Any non-zero speed
// request is mapped onto [MIN_DUTY, 1.0]. Tune this for your motors.
constexpr float MIN_DUTY = 0.25f;

// Flip these if a track runs the wrong way (instead of rewiring).
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = false;
}  // namespace motor

namespace drive {
// Joystick values below this magnitude are treated as zero.
constexpr float DEADBAND = 0.05f;
// Maximum change of track speed per second (1.0 = full speed).
// 5.0 means 0 -> full speed in 200 ms; protects gearboxes and the supply.
constexpr float ACCEL_PER_SECOND = 5.0f;
// Slowing down may be quicker, so the collision guard can stop in time.
constexpr float BRAKE_PER_SECOND = 15.0f;
// Motors stop if no command arrives for this long (lost Wi-Fi, closed tab...).
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
// Initial speed limit (0..1), adjustable from the web UI.
constexpr float DEFAULT_SPEED_LIMIT = 0.8f;
}  // namespace drive

namespace vehicle {
// Outer dimensions of the vehicle (tracks included).
constexpr float WIDTH_MM = 185.0f;
constexpr float LENGTH_MM = 220.0f;
}  // namespace vehicle

namespace lidar {
constexpr uint32_t BAUD_RATE = 230400;
constexpr size_t RX_BUFFER_BYTES = 4096;
// Mounting: the sensor's 0° mark points backwards on this vehicle.
constexpr float MOUNT_YAW_DEG = 180.0f;
// Set to false if obstacles show up mirrored (left/right swapped) in the map.
constexpr bool ANGLES_CLOCKWISE = true;
// Sensor position relative to the vehicle centre (+x right, +y forward).
constexpr float MOUNT_OFFSET_X_MM = 0.0f;
constexpr float MOUNT_OFFSET_Y_MM = 0.0f;
// Returns this close to the vehicle outline are the vehicle itself.
constexpr float FOOTPRINT_MARGIN_MM = 20.0f;
constexpr uint16_t MAX_RANGE_MM = 12000;
constexpr uint16_t MIN_SAMPLES_PER_REVOLUTION = 100;
// No complete scan for this long -> LiDAR considered offline.
constexpr uint32_t SCAN_TIMEOUT_MS = 600;
// Re-send the start command this often while no data arrives.
constexpr uint32_t RESTART_INTERVAL_MS = 2000;
}  // namespace lidar

namespace guard {
// The collision guard slows the vehicle down in the driving direction
// and stops it before it hits something.
constexpr bool ENABLED_BY_DEFAULT = true;
constexpr float STOP_DISTANCE_MM = 150.0f;   // no driving closer than this
constexpr float SLOW_DISTANCE_MM = 600.0f;   // start slowing down here
constexpr float SIDE_MARGIN_MM = 30.0f;      // extra width of the checked corridor
}  // namespace guard

namespace wifi {
// Station mode: join an existing network (left empty -> skipped).
constexpr const char* STA_SSID = WIFI_STA_SSID;
constexpr const char* STA_PASSWORD = WIFI_STA_PASSWORD;
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 15000;

// Fallback access point the vehicle opens itself (IP 192.168.4.1).
constexpr const char* AP_SSID = "TrackedVehicle";
constexpr const char* AP_PASSWORD = "drive1234";  // at least 8 characters

// mDNS name -> http://tank.local (where the phone supports mDNS)
constexpr const char* HOSTNAME = "tank";
}  // namespace wifi

namespace web {
constexpr uint16_t HTTP_PORT = 80;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 200;
// Minimum time between two LiDAR scans sent to the phone.
constexpr uint32_t SCAN_INTERVAL_MS = 150;
}  // namespace web

}  // namespace config
