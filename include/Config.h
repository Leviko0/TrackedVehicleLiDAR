#pragma once

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

// LiDAR UART (reserved, not used by the drive code yet)
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
constexpr float RAMP_PER_SECOND = 5.0f;
// Motors stop if no command arrives for this long (lost Wi-Fi, closed tab...).
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
// Initial speed limit (0..1), adjustable from the web UI.
constexpr float DEFAULT_SPEED_LIMIT = 0.8f;
}  // namespace drive

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
}  // namespace web

}  // namespace config
