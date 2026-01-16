#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
#include "subsystems/hardware/shooter.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{[&] {
    return container.control_input_.GetReadings().zero_bearing;
  }};
  drivetrain_zero_bearing_trigger.WhileTrue(frc2::InstantCommand([&] {
    container.drivetrain_.ZeroBearing();
  }).ToPtr());

  frc2::Trigger thirty_percent_trigger{[&] {
    return container.control_input_.GetReadings().thirty_percent_trigger;
  }};
  thirty_percent_trigger.OnTrue(frc2::InstantCommand([&] {
    container.shooter_.SetTarget(ShooterTarget{0.3});
  }).ToPtr());

  frc2::Trigger sixty_percent_trigger{[&] {
    return container.control_input_.GetReadings().sixty_percent_trigger;
  }};
  sixty_percent_trigger.OnTrue(frc2::InstantCommand([&] {
    container.shooter_.SetTarget(ShooterTarget{0.6});
  }).ToPtr());

  frc2::Trigger eightyfive_percent_trigger{[&] {
    return container.control_input_.GetReadings().eightyfive_percent_trigger;
  }};
  eightyfive_percent_trigger.OnTrue(frc2::InstantCommand([&] {
    container.shooter_.SetTarget(ShooterTarget{0.85});
  }).ToPtr());
}
