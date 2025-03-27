#include "commands/general/sim_drive.h"

SimDriveCommand::SimDriveCommand(
    RobotContainer &container, frc846::math::FieldPoint where)
    : frc846::robot::GenericCommand<RobotContainer, SimDriveCommand>{container,
          "sim_drive_command"},
      where_pos{where.point},
      where_bearing{where.bearing} {
  AddRequirements({&container_.drivetrain_});
  Log("Auto Going To: X {}, Y {}, T {}", where_pos[0], where_pos[1],
      where_bearing);
}

void SimDriveCommand::OnInit() {
  Log("Auto Going To: X {}, Y {}, T {}", where_pos[0], where_pos[1],
      where_bearing);
  container_.drivetrain_.SetSimPose(150_in, 100_in, 200_deg);
}

void SimDriveCommand::Periodic() {
  container_.drivetrain_.SetSimPose(50_in, 50_in, 200_deg);
}

void SimDriveCommand::OnEnd(bool interrupted) {}

bool SimDriveCommand::IsFinished() { return false; }