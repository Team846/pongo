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
                  getModifiedPrePose(numberOnRight, blueSide, leftSide), 13_fps,
                  22_fps_sq, 18_fps_sq},
              /*DriveToReefCommand{&(container.drivetrain_), is_left, false,
                  max_speed, max_acceleration, max_deceleration},*/
              frc2::ParallelRaceGroup{
                  frc2::WaitUntilCommand{[&] {
                    return !frc::RobotBase::IsSimulation() &&
                           !container.coral_ss_.coral_end_effector.GetReadings()
                                .has_piece_;
                  }},
                  frc846::robot::swerve::DriveToPointCommand{
                      &(container.drivetrain_),
                      ReefProvider::getReefScoringLocations(
                          false)[ReefProvider::getReefNumAuto(
                                     numberOnRight, leftSide)]
                          .mirror(blueSide),
                      10_fps, 22_fps_sq, 18_fps_sq, false},
                  frc2::WaitCommand{1_s}}}} {}

frc846::math::FieldPoint ReefAutoAutoAlignCommand::getModifiedPrePose(
    int numberOnRight, bool blueSide, bool leftSide) {
  frc846::math::FieldPoint pose = ReefProvider::getReefScoringLocations(
      false, true)[ReefProvider::getReefNumAuto(numberOnRight, leftSide)]
                                      .mirror(blueSide);
  pose.velocity = 4_fps;  // TODO: prefify this
  return pose;
}
