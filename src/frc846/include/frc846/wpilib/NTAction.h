#pragma once

#include <wpi/sendable/Sendable.h>

#include <functional>

namespace frc846::wpilib {

// Use with SmartDashboard::PutData to have buttons call functions on
// Shuffleboard.
class NTAction : public wpi::Sendable {
 public:
  NTAction(std::function<void()> callback);

  void InitSendable(wpi::SendableBuilder& builder);

 private:
  std::function<void()> callback_;
};

}  // namespace frc846::wpilib
