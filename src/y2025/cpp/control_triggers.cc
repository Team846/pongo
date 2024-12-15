#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container.control_input_.GetReadings().zero_bearing; }};

  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([&] {
        container.drivetrain_.SetBearing(
            frc846::util::ShareTables::GetBoolean("is_red_side") ? 0_deg
                                                                 : 180_deg);
      }).ToPtr());
}