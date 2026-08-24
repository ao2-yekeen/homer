#include "wheel_encoders.h"

#include "robot_config.h"

namespace homer {

WheelEncoders* WheelEncoders::instance_ = nullptr;

void WheelEncoders::begin() {
  pinMode(config::kRightEncoderA, INPUT);
  pinMode(config::kRightEncoderB, INPUT);
  pinMode(config::kLeftEncoderA, INPUT);
  pinMode(config::kLeftEncoderB, INPUT);
  instance_ = this;
  attachInterrupt(digitalPinToInterrupt(config::kRightEncoderA), onRightEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(config::kLeftEncoderA), onLeftEdge, RISING);
}

void WheelEncoders::readCounts(int32_t& left, int32_t& right) const {
  noInterrupts();
  left = left_count_;
  right = right_count_;
  interrupts();
}

void IRAM_ATTR WheelEncoders::onRightEdge() {
  if (instance_ != nullptr) instance_->countRight();
}

void IRAM_ATTR WheelEncoders::onLeftEdge() {
  if (instance_ != nullptr) instance_->countLeft();
}

void IRAM_ATTR WheelEncoders::countRight() {
  right_count_ += digitalRead(config::kRightEncoderB) ? 1 : -1;
}

void IRAM_ATTR WheelEncoders::countLeft() {
  left_count_ += digitalRead(config::kLeftEncoderB) ? -1 : 1;
}

}  // namespace homer
