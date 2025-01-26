#include "commands/teleop/drive_to_reef_command.h"

#include "reef.h"

DriveToReefCommand::DriveToReefCommand(RobotContainer& container, bool is_left)
    : frc846::robot::GenericCommandGroup<RobotContainer, DriveToReefCommand,
          frc2::SequentialCommandGroup>{container, "DriveToReefCommand",
          frc2::SequentialCommandGroup{
              std::move(getCommandChain(container, is_left))}} {
  AddRequirements({&container.drivetrain_});
}

std::vector<std::unique_ptr<frc2::Command>> DriveToReefCommand::getCommandChain(
    RobotContainer& container, bool is_left) {
  std::vector<std::unique_ptr<frc2::Command>> cmdChain;
  auto pos = container.drivetrain_.GetReadings().estimated_pose.position;
  int reef_target_pos = ReefProvider::getClosestReefSide(pos);

  auto target_pos =
      ReefProvider::getReefScoringLocations()[2 * reef_target_pos +
                                              (is_left ? 0 : 1)];
  cmdChain.push_back(
      std::make_unique<frc846::robot::swerve::DriveToPointCommand>(
          &container.drivetrain_,
          frc846::math::FieldPoint(target_pos.point, target_pos.bearing, 2_fps),
          15_fps, 35_fps_sq, 15_fps_sq));  // TODO: Prefify
  cmdChain.push_back(std::make_unique<LockToPointCommand>(container,
      frc846::math::FieldPoint(target_pos.point, target_pos.bearing, 0_fps)));
  return cmdChain;
}
