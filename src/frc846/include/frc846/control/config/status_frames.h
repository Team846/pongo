#pragma once

namespace frc846::control::config {

/*
StatusFrame

Enum with types representing the status frame categories for common motor
controllers.
*/
enum StatusFrame {
  kPositionFrame,
  kVelocityFrame,
  kCurrentFrame,
  kFaultFrame,
  kSensorFrame,
  kAbsoluteFrame,
  kLeader
};

}  // namespace frc846::control::config