#include "frc846/wpilib/NTAction.h"

#include <wpi/sendable/SendableBuilder.h>

namespace frc846::wpilib {

NTAction::NTAction(std::function<void()> callback) : callback_(callback) {}
void NTAction::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Command");
  builder.AddStringProperty(".name", [] { return "Run"; }, nullptr);
  builder.AddBooleanProperty(
      "running", [] { return false; },
      [this](bool value) {
        if (value) { callback_(); }
      });
}

}  // namespace frc846::wpilib