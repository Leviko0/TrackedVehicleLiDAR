#include "CommandParser.h"

#include <cmath>
#include <cstdlib>

#include "DriveMath.h"

namespace drivecore {

namespace {

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

void skipSpaces(const char*& p) {
  while (isSpace(*p)) ++p;
}

bool readFloat(const char*& p, float& out) {
  skipSpaces(p);
  char* end = nullptr;
  const float value = std::strtof(p, &end);
  if (end == p || !std::isfinite(value)) return false;
  out = value;
  p = end;
  return true;
}

bool atEnd(const char* p) {
  skipSpaces(p);
  return *p == '\0';
}

}  // namespace

Command parseCommand(const char* text) {
  Command cmd;
  if (text == nullptr) return cmd;

  const char* p = text;
  skipSpaces(p);
  const char op = *p;
  if (op == '\0') return cmd;
  ++p;

  switch (op) {
    case 'D': {
      float throttle = 0.0f;
      float turn = 0.0f;
      if (readFloat(p, throttle) && readFloat(p, turn) && atEnd(p)) {
        cmd.type = CommandType::Drive;
        cmd.throttle = clampUnit(throttle);
        cmd.turn = clampUnit(turn);
      }
      break;
    }
    case 'L': {
      float limit = 0.0f;
      if (readFloat(p, limit) && atEnd(p)) {
        cmd.type = CommandType::SpeedLimit;
        cmd.value = clamp(limit, 0.0f, 1.0f);
      }
      break;
    }
    case 'G': {
      float enabled = 0.0f;
      if (readFloat(p, enabled) && atEnd(p) && (enabled == 0.0f || enabled == 1.0f)) {
        cmd.type = CommandType::Guard;
        cmd.value = enabled;
      }
      break;
    }
    case 'S':
      if (atEnd(p)) cmd.type = CommandType::Stop;
      break;
    case 'P':
      if (atEnd(p)) cmd.type = CommandType::Ping;
      break;
    default:
      break;
  }
  return cmd;
}

}  // namespace drivecore
