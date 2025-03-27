#include "commands/general/reef_auto_auto_align.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "commands/teleop/drive_to_reef_command.h"
#include "commands/teleop/lock_to_reef_command.h"
#include "frc846/robot/swerve/wait_until_close.h"
#include "reef.h"

ReefAutoAutoAlignCommand::ReefAutoAutoAlignCommand(
    RobotContainer& container, int numberOnRight, bool blueSide, bool leftSide)
    : GenericCommandGroup<RobotContainer, ReefAutoAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "reef_auto_align",
          frc2::SequentialCommandGroup{
              frc846::robot::swerve::DriveToPointCommand{
                  &(container.drivetrain_),
                  ReefProvider::getReefScoringLocations(
                      false, true)[ReefProvider::getReefNumAuto(
                                       numberOnRight, leftSide)]
                      .mirror(blueSide),
                  10_fps, 15_fps_sq, 15_fps_sq},
              /*DriveToReefCommand{&(container.drivetrain_), is_left, false,
                  max_speed, max_acceleration, max_deceleration},*/
              frc2::ParallelRaceGroup{
                  frc2::WaitUntilCommand{[&] {
                    return !container.coral_ss_.coral_end_effector.GetReadings()
                                .has_piece_;
                  }},
                  frc846::robot::swerve::WaitUntilClose{
                      &(container.drivetrain_),
                      ReefProvider::getReefScoringLocations(
                          false)[ReefProvider::getReefNumAuto(
                                     numberOnRight, leftSide)]
                          .mirror(blueSide)},
                  frc846::robot::swerve::LockToPointCommand{
                      &(container.drivetrain_),
                      ReefProvider::getReefScoringLocations(
                          false)[ReefProvider::getReefNumAuto(
                                     numberOnRight, leftSide)]
                          .mirror(blueSide)},
                  frc2::WaitCommand{5_s}}}} {}
