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

  // FAKE, TODO: remove

  frc2::Trigger test_move_10_ft_trigger{[&] {
    return container.control_input_.GetReadings().test_move_10_ft;
  }};

  test_move_10_ft_trigger.WhileTrue(
      frc846::robot::swerve::DriveToPointCommand{&container.drivetrain_,
          {{0_ft, 9_ft}, 0_deg, 0_fps}, 15_fps, 35_fps_sq, 15_fps_sq}
          .ToPtr());

  frc2::Trigger test_bearing_pid_trigger{[&] {
    return container.control_input_.GetReadings().test_bearing_pid;
  }};

  test_bearing_pid_trigger.WhileTrue(
      frc846::robot::swerve::AimCommand{&container.drivetrain_, 0_deg}.ToPtr());

  frc2::Trigger test_lock_trigger{[&] {
    return container.control_input_.GetReadings().test_lock;
  }};

  test_lock_trigger.WhileTrue(frc846::robot::swerve::LockToPointCommand{
      &container.drivetrain_,
      {{0_ft, 0_ft}, 0_deg,
          0_fps}}.ToPtr());

  // END FAKE
}