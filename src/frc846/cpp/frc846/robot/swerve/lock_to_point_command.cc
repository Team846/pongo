#include "frc846/robot/swerve/lock_to_point_command.h"

namespace frc846::robot::swerve {

LockToPointCommand::LockToPointCommand(
    DrivetrainSubsystem* drivetrain, frc846::math::FieldPoint target)
    : Loggable("LockToPointCommand"), drivetrain_(drivetrain), target_(target) {
  SetName("LockToPointCommand");
  AddRequirements({drivetrain_});
}

void LockToPointCommand::Initialize() { Log("LockToPointCommand initialized"); }

void LockToPointCommand::Execute() {
  auto pos = drivetrain_->GetReadings().pose.position;

  if (auto [target, is_valid] = GetTargetPoint(); is_valid) target_ = target;

  frc846 ::math::Vector2D r_vec = target_.point - pos;
  Graph("lock_to_point/x_err", r_vec[0]);
  Graph("lock_to_point/y_err", r_vec[1]);
  if (r_vec.magnitude() <
      drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "lock_gains/deadband")) {
    drivetrain_->SetTarget(frc846::robot::swerve::DrivetrainOLControlTarget{
        {0_fps, 0_fps}, drivetrain_->ApplyBearingPID(target_.bearing)});
  } else {
    frc846::control::base::MotorGains lock_gains{
        drivetrain_->GetPreferenceValue_double("lock_gains/_kP"),
        drivetrain_->GetPreferenceValue_double("lock_gains/_kI"),
        drivetrain_->GetPreferenceValue_double("lock_gains/_kD"), 0.0};
    units::feet_per_second_t speed_target =
        1_fps *
        lock_gains.calculate(r_vec.magnitude().to<double>(), 0.0,
            drivetrain_->GetReadings().pose.velocity.magnitude().to<double>(),
            0.0);

    drivetrain_->SetTarget(frc846::robot::swerve::DrivetrainOLControlTarget{
        frc846::math::VectorND<units::feet_per_second, 2>{
            speed_target, r_vec.angle(true) + 180_deg, true},
        drivetrain_->ApplyBearingPID(target_.bearing)});
  }
}

void LockToPointCommand::End(bool interrupted) { drivetrain_->SetTargetZero(); }

bool LockToPointCommand::IsFinished() { return false; }

}  // namespace frc846::robot::swerve