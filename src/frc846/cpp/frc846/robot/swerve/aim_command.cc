#include "frc846/robot/swerve/aim_command.h"

namespace frc846::robot::swerve {

AimCommand::AimCommand(DrivetrainSubsystem* drivetrain, units::degree_t target)
    : Loggable("AimCommand"), drivetrain_{drivetrain}, target_(target) {
  SetName("AimCommand");
  AddRequirements({drivetrain_});
}

void AimCommand::Initialize() {
  Log("AimCommand initialized with target {}", target_);
}

void AimCommand::Execute() {
  drivetrain_->SetTarget(DrivetrainOLControlTarget{
      {0_fps, 0_fps}, drivetrain_->ApplyBearingPID(target_)});
}

void AimCommand::End(bool interrupted) {
  Log("AimCommand ended with interruption status {}", interrupted);
  drivetrain_->SetTargetZero();
}

bool AimCommand::IsFinished() { return false; }

}  // namespace frc846::robot::swerve