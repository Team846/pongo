#include "frc846/robot/swerve/wait_until_close.h"

namespace frc846::robot::swerve {

WaitUntilClose::WaitUntilClose(
    DrivetrainSubsystem* drivetrain, frc846::math::FieldPoint target)
    : Loggable("WaitUntilClose"), drivetrain_(drivetrain), target_(target) {
  SetName("WaitUntilClose");
  AddRequirements({});
}

void WaitUntilClose::Initialize() { Log("WaitUntilClose initialized"); }

void WaitUntilClose::Execute() {}

void WaitUntilClose::End(bool interrupted) {}

bool WaitUntilClose::IsFinished() {
  return (drivetrain_->GetReadings().estimated_pose.position - target_.point)
             .magnitude() < .35_in;
}

}  // namespace frc846::robot::swerve