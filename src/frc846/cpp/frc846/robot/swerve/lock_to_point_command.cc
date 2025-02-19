#include "frc846/robot/swerve/lock_to_point_command.h"

#include "subsystems/robot_container.h"

namespace frc846::robot::swerve {

LockToPointCommand::LockToPointCommand(DrivetrainSubsystem* drivetrain,
    frc846::math::FieldPoint target,
    std::function<std::pair<frc846::math::FieldPoint, bool>(
        frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
        bool firstLoop)>
        updateTarget)
    : Loggable("LockToPointCommand"),
      drivetrain_(drivetrain),
      target_(target),
      updateTarget_(updateTarget) {
  SetName("LockToPointCommand");
  AddRequirements({drivetrain_});
}

void LockToPointCommand::Initialize() {
  Log("LockToPointCommand initialized");
  start_ = frc846::math::FieldPoint{
      drivetrain_->GetReadings().estimated_pose.position,
      drivetrain_->GetReadings().pose.bearing,
      drivetrain_->GetReadings().estimated_pose.velocity.magnitude()};
  first_loop_ = true;
}

void LockToPointCommand::Execute() {
  auto pos = drivetrain_->GetReadings().estimated_pose.position;

  if (updateTarget_ != nullptr) {
    const auto [target, is_valid] = updateTarget_(target_, start_, first_loop_);
    Graph("overriding_target", is_valid);
    if (is_valid) {
      target_ = target;
      first_loop_ = false;
    }
    Graph("update_target_null", false);
  } else {
    Graph("overriding_target", false);
    Graph("update_target_null", true);
  }

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
        1_fps * lock_gains.calculate(r_vec.magnitude().to<double>(), 0.0,
                    drivetrain_->GetReadings()
                        .estimated_pose.velocity.magnitude()
                        .to<double>(),
                    0.0);
    speed_target = units::math::min(units::math::max(speed_target,
        -drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
            "lock_max_speed")), drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
            "lock_max_speed"));

    drivetrain_->SetTarget(frc846::robot::swerve::DrivetrainOLControlTarget{
        frc846::math::VectorND<units::feet_per_second, 2>{
            speed_target, r_vec.angle(true) + 180_deg, true},
        drivetrain_->ApplyBearingPID(target_.bearing)});
  }
}
void LockToPointCommand::End(bool interrupted) { drivetrain_->SetTargetZero(); }

bool LockToPointCommand::IsFinished() { return false; }

}  // namespace frc846::robot::swerve

//deadband .5, 
//p -2
//d .01