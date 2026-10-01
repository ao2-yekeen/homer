#include "serial_link.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../config/robot_config.h"

namespace homer {
namespace {

uint16_t crc16(const char* data, size_t length) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint8_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

bool parseLong(const char* text, long min, long max, long& result) {
  if (!text || !*text) return false;
  char* end = nullptr;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno || *end || value < min || value > max) return false;
  result = value;
  return true;
}

bool parseFloat(const char* text, float& result) {
  if (!text || !*text) return false;
  char* end = nullptr;
  errno = 0;
  const float value = strtof(text, &end);
  if (errno || *end || !isfinite(value) || fabsf(value) > 5.0f) return false;
  result = value;
  return true;
}

}  // namespace

void SerialLink::begin() {
  Serial.begin(config::kSerialBaud);
  boot_id_ = esp_random();
  last_ready_ms_ = millis() - 1000U;
}

void SerialLink::spin(uint32_t now_ms) {
  // Bound parser work so malformed input cannot delay the motor dead-man.
  for (unsigned i = 0; i < 96 && Serial.available() > 0; ++i)
    receiveByte(static_cast<char>(Serial.read()), millis());
  if (now_ms - last_ready_ms_ >= 1000U) {
    last_ready_ms_ = now_ms;
    char body[48];
    snprintf(body, sizeof(body), "R|%lu", static_cast<unsigned long>(boot_id_));
    send(body);
  }
  if (config::kEnableEncoderOdometry &&
      now_ms - last_odom_ms_ >= config::kOdometryPeriodMs) {
    last_odom_ms_ = now_ms;
    publishOdometry(now_ms);
  }
  if (now_ms - last_status_ms_ >= 200U) {
    last_status_ms_ = now_ms;
    publishStatus(now_ms);
  }
}

void SerialLink::receiveByte(char byte, uint32_t now_ms) {
  if (byte == '\n') {
    if (!discarding_ && input_size_) handleFrame(now_ms);
    input_size_ = 0;
    discarding_ = false;
    return;
  }
  if (discarding_) return;
  if (byte == '\r') return;
  if (input_size_ >= sizeof(input_) - 1) {
    input_size_ = 0;
    discarding_ = true;
    return;
  }
  input_[input_size_++] = byte;
}

void SerialLink::handleFrame(uint32_t now_ms) {
  input_[input_size_] = '\0';
  char* checksum = strrchr(input_, '|');
  if (!checksum || strlen(checksum + 1) != 4) return;
  char* end = nullptr;
  const unsigned long expected = strtoul(checksum + 1, &end, 16);
  if (*end || expected > 0xffff ||
      crc16(input_, checksum - input_) != expected) return;
  *checksum = '\0';
  char* save = nullptr;
  char* version = strtok_r(input_, "|", &save);
  char* type = strtok_r(nullptr, "|", &save);
  char* seq = strtok_r(nullptr, "|", &save);
  long sequence = 0;
  if (!version || strcmp(version, "HOM1") || !type || strlen(type) != 1 ||
      !parseLong(seq, 0, 65535, sequence)) return;
  char* first = strtok_r(nullptr, "|", &save);
  char* second = strtok_r(nullptr, "|", &save);
  if (strtok_r(nullptr, "|", &save)) return;
  long value = 0;
  if (type[0] == 'S' && !first) {
    robot_.stopDrive();
  } else if (type[0] == 'M' && !second && parseLong(first, 0, 1, value)) {
    robot_.setMode(value ? RobotMode::kAutonomous : RobotMode::kTeleop, now_ms);
  } else if (type[0] == 'V' && !second && parseLong(first, -1, 2, value)) {
    if (value == -1) robot_.clearModeOverride(now_ms);
    else robot_.setModeOverride(static_cast<RobotMode>(value), now_ms);
  } else if (type[0] == 'N' && !second &&
             parseLong(first, config::kNeckMinimumAngle,
                       config::kNeckMaximumAngle, value)) {
    robot_.setNeckAngle(static_cast<int>(value));
  } else if ((type[0] == 'T' || type[0] == 'A') && second) {
    float linear = 0.0f, angular = 0.0f;
    if (!parseFloat(first, linear) || !parseFloat(second, angular)) return;
    DriveCommand command;
    command.linear = linear;
    command.angular = angular;
    if (type[0] == 'A') {
      robot_.receiveOverrideDriveCommand(command, now_ms);
      robot_.receiveDriveCommand(RobotMode::kAutonomous, command, now_ms);
    } else {
      robot_.receiveDriveCommand(RobotMode::kTeleop, command, now_ms);
    }
  }
}

void SerialLink::send(const char* body) {
  char frame[160];
  const int prefix = snprintf(frame, sizeof(frame), "HOM1|%c|%u%s", body[0],
                              static_cast<unsigned>(tx_sequence_++), body + 1);
  if (prefix < 0 || prefix >= static_cast<int>(sizeof(frame) - 7)) return;
  const uint16_t crc = crc16(frame, prefix);
  const int length = snprintf(frame + prefix, sizeof(frame) - prefix,
                              "|%04X\n", static_cast<unsigned>(crc));
  if (length < 0 || prefix + length >= static_cast<int>(sizeof(frame))) return;
  if (Serial.availableForWrite() >= prefix + length)
    Serial.write(reinterpret_cast<const uint8_t*>(frame), prefix + length);
}

void SerialLink::publishOdometry(uint32_t now_ms) {
  const OdometryState& s = robot_.odometry();
  char body[120];
  snprintf(body, sizeof(body), "O|%lu|%.4f|%.4f|%.5f|%.4f|%.4f",
           static_cast<unsigned long>(now_ms), s.x, s.y, s.heading,
           s.linear_velocity, s.angular_velocity);
  send(body);
}

void SerialLink::publishStatus(uint32_t now_ms) {
  const DriveCommand command = robot_.lastDriveCommand();
  char body[100];
  snprintf(body, sizeof(body), "D|%lu|%.4f|%u|%u|%.4f|%u|%lu",
           static_cast<unsigned long>(now_ms), command.linear,
           ledcRead(config::kLeftMotorPwmChannel),
           ledcRead(config::kRightMotorPwmChannel), command.angular,
           static_cast<unsigned>(robot_.mode()),
           static_cast<unsigned long>(now_ms - robot_.lastActiveCommandMs()));
  send(body);
}

}  // namespace homer
