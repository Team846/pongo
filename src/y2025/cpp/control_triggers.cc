#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/general/coral_position_command.h"
#include "commands/teleop/complete_gpd_command.h"
#include "commands/teleop/gpd_ss_command.h"
#include "commands/teleop/lock_gpd_command.h"
#include "commands/teleop/lock_to_reef_command.h"
#include "commands/teleop/net_auto_align.h"
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
  }}.WhileTrue(ReefAutoAlignCommand{container, true, 13_fps, 4_fps, 25_fps_sq,
      10_fps_sq, container.control_input_.base_adj}
                   .ToPtr());
  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().lock_right_reef;
  }}.WhileTrue(ReefAutoAlignCommand{container, false, 13_fps, 4_fps, 25_fps_sq,
      10_fps_sq, container.control_input_.base_adj}
                   .ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().lock_net;
  }}.WhileTrue(NetAutoAlignCommand{
      container, 13_fps, 10_fps, 25_fps_sq, 10_fps_sq, 25_fps_sq, 10_fps_sq}
                   .ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().targeting_algae &&

           !container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }}.OnTrue(GPDSSCommand{container}.Until([&] {
    return !container.control_input_.GetReadings().targeting_algae ||
           container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  }));

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().flick ||
           container.coral_ss_.GetReadings().auto_flick_valid;
  }}.OnTrue(frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.13_s},
      CoralPositionCommand{container, kCoral_FLICK,
          true}}.ToPtr());

  frc2::Trigger{[&] {
    return container.control_input_.GetReadings().auto_pick_used;
  }}.OnTrue(frc2::InstantCommand([&] {
    container.control_input_.SetTarget({false, true});
  })
                .AndThen(frc2::WaitCommand(0.5_s).ToPtr())
                .AndThen(frc2::InstantCommand([&] {
                  container.control_input_.SetTarget({false, false});
                }).ToPtr()));

  frc2::Trigger{[&] {
    return container.coral_ss_.GetReadings().piece_entered ||
           container.algal_ss_.GetReadings().has_piece;
  }}.OnTrue(frc2::InstantCommand([&] {
    container.control_input_.SetTarget({true, false});
  })
                .AndThen(frc2::WaitCommand(0.5_s).ToPtr())
                .AndThen(frc2::InstantCommand([&] {
                  container.control_input_.SetTarget({false, false});
                }).ToPtr()));

  // frc2::Trigger{[&] {
  //   return container.control_input_.GetReadings().targeting_algae &&
  //          container.GPD_.GetReadings().gamepieces.size() != 0U &&
  //          !container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  // }}.OnTrue(LockGPDCommand{container}.Until([&] {
  //   return !container.control_input_.GetReadings().targeting_algae ||
  //          container.algal_ss_.algal_end_effector.GetReadings().has_piece_;
  // }));
}
