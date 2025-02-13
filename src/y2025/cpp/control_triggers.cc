#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/teleop/complete_gpd_command.h"
#include "commands/teleop/lock_to_reef_command.h"
#include "commands/teleop/reef_auto_align.h"
#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "reef.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{[&] {
    return container.control_input_.GetReadings().zero_bearing;
  }};
  drivetrain_zero_bearing_trigger.WhileTrue(frc2::InstantCommand([&] {
    container.drivetrain_.ZeroBearing();
  }).ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().lock_left_reef;
  }}.WhileTrue(ReefAutoAlignCommand{
      container, true, 5_fps, 15_fps_sq, 15_fps_sq}
                   .ToPtr());
  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().lock_right_reef;
  }}.WhileTrue(ReefAutoAlignCommand{
      container, false, 5_fps, 15_fps_sq, 15_fps_sq}
                   .ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().targeting_algae &&
           container.GPD_.GetReadings().gamepieces.size() != 0U &&
           !container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }}.OnTrue(CompleteGPDCommand{container}.Until([&] {
    return !container.control_input_.GetReadings().targeting_algae ||
           container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }));
}
