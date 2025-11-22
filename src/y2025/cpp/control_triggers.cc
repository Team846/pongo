#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "frc846/robot/swerve/lock_to_point_command.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{[&] {
    return container.control_input_.GetReadings().zero_bearing;
  }};
  drivetrain_zero_bearing_trigger.WhileTrue(frc2::InstantCommand([&] {
    container.drivetrain_.ZeroBearing();
  }).ToPtr());

  frc2::Trigger ictest_x_button_trigger{[&] {
    return container.control_input_.GetReadings().ictest_x_button;
  }};
  ictest_x_button_trigger.OnTrue(frc2::InstantCommand([&] {
    ICTestTarget target;
    target.pos = units::degree_t(units::radian_t(-10.0));
    container.ictest_.SetTarget(target);
  }).ToPtr());

  frc2::Trigger ictest_y_button_trigger{[&] {
    return container.control_input_.GetReadings().ictest_y_button;
  }};
  ictest_y_button_trigger.OnTrue(frc2::InstantCommand([&] {
    ICTestTarget target;
    target.pos = units::degree_t(units::radian_t(10.0));
    container.ictest_.SetTarget(target);
  }).ToPtr());
}
