#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/teleop/complete_gpd_command.h"
#include "commands/teleop/gpd_ss_command.h"
#include "commands/teleop/lock_gpd_command.h"
#include "commands/teleop/lock_to_reef_command.h"
#include "commands/teleop/reef_auto_align.h"
#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
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
  }}.WhileTrue(ReefAutoAlignCommand{container, true,
      container.drivetrain_
          .GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "lock_max_speed"),
      35_fps_sq, 15_fps_sq, container.control_input_.base_adj}
                   .ToPtr());
  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().lock_right_reef;
  }}.WhileTrue(ReefAutoAlignCommand{container, false,
      container.drivetrain_
          .GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "lock_max_speed"),
      35_fps_sq, 15_fps_sq, container.control_input_.base_adj}
                   .ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().targeting_algae &&

           !container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }}.OnTrue(GPDSSCommand{container}.Until([&] {
    return !container.control_input_.GetReadings().targeting_algae ||
           container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }));

  // frc2::Trigger{[&] {
  //   return container.control_input_.GetReadings().targeting_algae &&
  //          container.GPD_.GetReadings().gamepieces.size() != 0U &&
  //          !container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  // }}.OnTrue(LockGPDCommand{container}.Until([&] {
  //   return !container.control_input_.GetReadings().targeting_algae ||
  //          container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  // }));
}
