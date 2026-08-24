#pragma once

namespace homer {

struct DriveCommand {
  float linear = 0.0f;
  float angular = 0.0f;
};

class DifferentialDrive {
 public:
  void begin();
  void apply(const DriveCommand& command);
  void stop();

 private:
  void setWheelDuty(int left, int right);
};

}  // namespace homer
