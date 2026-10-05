#pragma once

// Text protocol spoken between the web UI and the vehicle (over WebSocket).
//
//   "D <throttle> <turn>"  drive, both values in [-1, 1]
//   "S"                    stop immediately
//   "L <limit>"            set the speed limit, value in [0, 1]
//   "G <0|1>"              collision guard off / on
//   "P"                    ping / keep-alive
//
// Out-of-range numbers are clamped, malformed messages yield Invalid.

namespace drivecore {

enum class CommandType { Invalid, Drive, Stop, SpeedLimit, Guard, Ping };

struct Command {
  CommandType type = CommandType::Invalid;
  float throttle = 0.0f;  // Drive
  float turn = 0.0f;      // Drive
  float value = 0.0f;     // SpeedLimit, Guard (0 or 1)
};

Command parseCommand(const char* text);

}  // namespace drivecore
