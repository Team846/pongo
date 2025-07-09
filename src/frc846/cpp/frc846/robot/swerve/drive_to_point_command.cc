#include "frc846/robot/swerve/drive_to_point_command.h"

#include <frc/RobotBase.h>

namespace frc846::robot::swerve {

DriveToPointCommand::DriveToPointCommand(DrivetrainSubsystem* drivetrain,
    frc846::math::FieldPoint target, units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration, bool end_when_close)
    : Loggable("DriveToPointCommand"),
      drivetrain_(drivetrain),
      max_speed_(max_speed),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration),
      target_(target),
      end_when_close_(end_when_close) {
  SetName("DriveToPointCommand");
  AddRequirements({drivetrain_});
  if (end_when_close_) { target.velocity = 0_fps; }
}

void DriveToPointCommand::Initialize() {
  Log("DriveToPointCommand initialized");
  start_point_ = drivetrain_->GetReadings().estimated_pose.position;

  const auto [new_target_point, is_valid] = GetTargetPoint();
  if (is_valid) target_ = new_target_point;
}

#define kC drivetrain_->GetPreferenceValue_double("drive_to_point/kC")
#define kA drivetrain_->GetPreferenceValue_double("drive_to_point/kA")
#define kE drivetrain_->GetPreferenceValue_double("drive_to_point/kE")

void DriveToPointCommand::Execute() {
  DrivetrainReadings dt_readings{drivetrain_->GetReadings()};

  const auto [new_target_point, is_valid] = GetTargetPoint();

  DrivetrainOLControlTarget dt_target{{0_fps, 0_fps}, 0_deg_per_s};

  frc846::math::Vector2D delta_vec =
      target_.point - dt_readings.estimated_pose.position;

  units::degree_t heading_direction = delta_vec.angle(true);
  if (dt_readings.estimated_pose.velocity.magnitude() > 2.0_fps)
    heading_direction = dt_readings.estimated_pose.velocity.angle(true);

  units::degree_t O = delta_vec.angle(true) - heading_direction;

  units::feet_per_second_t vel_dir_target =
      units::math::min(max_speed_,
          target_.velocity + units::math::sqrt(delta_vec.magnitude() * kE *
                                               max_deceleration_)) *
      units::math::cos(O);

  units::feet_per_second_t vel_lat_target =
      units::math::sin(O) *
      units::math::sqrt(units::math::max(5_fps,
                            dt_readings.estimated_pose.velocity.magnitude()) *
                        1_fps) *
      kC;

  vel_lat_target = units::math::min(
      5_fps, units::math::max(-5_fps, vel_lat_target));  // TODO: prefify

  units::feet_per_second_t vel_dir_target_controlled =
      vel_dir_target +
      (vel_dir_target - dt_readings.estimated_pose.velocity.magnitude()) * kA;

  vel_dir_target_controlled = units::math::min(
      12.5_fps, units::math::max(-12.5_fps,
                    vel_dir_target_controlled));  // TODO: prefify

  frc846::math::VectorND<units::feet_per_second, 2> vel_target{
      vel_lat_target, vel_dir_target_controlled};
  vel_target = vel_target.rotate(heading_direction, true);

  dt_target.velocity = vel_target;
  dt_target.angular_velocity = drivetrain_->ApplyBearingPID(target_.bearing);
  dt_target.cut_excess_steering = true;
  dt_target.accel_clamp = max_acceleration_;

  if (delta_vec.magnitude() < 0.75_in) { dt_target.velocity = {0_fps, 0_fps}; }

  drivetrain_->SetTarget(dt_target);
}

void DriveToPointCommand::End(bool interrupted) {
  Log("DriveToPointCommand ended with interruption status {} at position "
      "x: {} "
      "and position y: {}",
      interrupted, drivetrain_->GetReadings().estimated_pose.position[0],
      drivetrain_->GetReadings().estimated_pose.position[1]);

  Log("The error at the end is x: {}, y:{}",
      target_.point[0] - drivetrain_->GetReadings().estimated_pose.position[0],
      target_.point[1] - drivetrain_->GetReadings().estimated_pose.position[1]);
  drivetrain_->SetTargetZero();
}

bool DriveToPointCommand::IsFinished() {
  auto drivetrain_readings = drivetrain_->GetReadings();
  auto current_point = drivetrain_readings.estimated_pose.position;

  //   if (drivetrain_readings.pose.velocity.magnitude() <
  //       drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
  //           "vel_stopped_thresh")) {
  //     num_stalled_loops_ += 1;
  //   } else {
  //     num_stalled_loops_ = 0;
  //   }

  // TODO: add back bump detection
  // Consider: predicting time command should take. Timeout if much past
  // that.

  return end_when_close_ &&
         ((current_point - start_point_).magnitude() >=
                 (target_.point - start_point_).magnitude() ||
             (target_.point - current_point).magnitude() <
                 drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
                     "drive_to_point/threshold"));
}

}  // namespace frc846::robot::swerve