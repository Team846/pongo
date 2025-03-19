#include "frc846/robot/swerve/drive_to_point_command.h"

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
}

void DriveToPointCommand::Initialize() {
  Log("DriveToPointCommand initialized");
  start_point_ = drivetrain_->GetReadings().estimated_pose.position;
  is_decelerating_ = false;
  num_stalled_loops_ = 0;

  const auto [new_target_point, is_valid] = GetTargetPoint();
  if (is_valid) target_ = new_target_point;

  direction_offset = (target_.point - start_point_).angle();
}

void DriveToPointCommand::Execute() {
  DrivetrainReadings dt_readings{drivetrain_->GetReadings()};

  const auto [new_target_point, is_valid] = GetTargetPoint();

  Graph("override_is_valid", is_valid);
  if (is_valid) target_ = new_target_point;

  DrivetrainAccelerationControlTarget dt_target{
      .linear_acceleration = max_acceleration_,
      .accel_dir = (target_.point - start_point_).angle(true),
      .angular_velocity = 0_deg_per_s,
  };

  Graph("max_deceleration", max_deceleration_);

  // Motion Profiling in direction of target point

  double DIRECTIONAL_GAIN = 0.9;  // TODO: prefify
  units::feet_per_second_squared_t directional_max_dcl =
      max_deceleration_ * DIRECTIONAL_GAIN;
  units::feet_per_second_squared_t directional_max_accl =
      max_acceleration_ * DIRECTIONAL_GAIN;

  units::feet_per_second_t directional_velocity =
      units::math::cos(
          dt_readings.estimated_pose.velocity.angle() - direction_offset) *
      dt_readings.estimated_pose.velocity.magnitude();

  units::second_t t_decel_directional =
      (directional_velocity - target_.velocity) / directional_max_dcl;
  units::foot_t stopping_distance_dir =
      units::math::abs(directional_velocity * t_decel_directional) -
      ((directional_max_dcl * t_decel_directional * t_decel_directional) / 2.0);

  units::foot_t dist_to_target_directional =
      (target_.point - dt_readings.estimated_pose.position).magnitude() *
          units::math::cos(
              (target_.point - dt_readings.estimated_pose.position).angle() -
              direction_offset) -
      drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "drive_to_subtract");

  Graph("dir_velocity_dir", directional_velocity);

  frc846::math::VectorND<units::feet_per_second_squared, 2> directional_accl;

  if (dist_to_target_directional <= stopping_distance_dir) {
    is_decelerating_ = true;

    if (dist_to_target_directional < 4_in)
      directional_accl =
          frc846::math::VectorND<units::feet_per_second_squared, 2>(
              directional_max_dcl * units::math::abs(stopping_distance_dir) /
                  units::math::abs(dist_to_target_directional),
              direction_offset + 180_deg, true);
    // dt_target.linear_acceleration = max_deceleration_ *
    //                                 units::math::abs(stopping_distance) /
    //                                 units::math::abs(dist_to_target_directional);
    else
      directional_accl =
          frc846::math::VectorND<units::feet_per_second_squared, 2>(
              directional_max_dcl, direction_offset + 180_deg, true);
  } else {
    is_decelerating_ = false;
    if (directional_velocity < DIRECTIONAL_GAIN * max_speed_) {
      directional_accl =
          frc846::math::VectorND<units::feet_per_second_squared, 2>(
              directional_max_accl, direction_offset, true);
    } else
      directional_accl = {0_fps_sq, 0_fps_sq};
  }

  // Motion Profiling in Corrective way
  double CORRECTIONAL_GAIN = 0.4;
  units::feet_per_second_squared_t corr_max_dcl =
      max_deceleration_ * CORRECTIONAL_GAIN;
  units::feet_per_second_squared_t corr_max_accl =
      max_acceleration_ * CORRECTIONAL_GAIN;

  units::feet_per_second_t corr_velocity =
      units::math::sin(
          dt_readings.estimated_pose.velocity.angle() - direction_offset) *
      dt_readings.estimated_pose.velocity.magnitude();

  units::second_t t_decel_corr = units::math::abs(corr_velocity) / corr_max_dcl;
  units::foot_t stopping_distance_corr =
      units::math::abs(corr_velocity * t_decel_corr) -
      ((corr_max_dcl * t_decel_corr * t_decel_corr) / 2.0);

  units::foot_t dist_to_target_corr =
      (target_.point - dt_readings.estimated_pose.position).magnitude() *
      units::math::sin(
          (target_.point - dt_readings.estimated_pose.position).angle() -
          direction_offset) *
      (drivetrain_->GetPreferenceValue_double("drive_correctional_gain") *
          max_speed_.to<double>());

  frc846::math::VectorND<units::feet_per_second_squared, 2> corr_accl;

  if (units::math::abs(dist_to_target_corr) <= stopping_distance_corr) {
    corr_accl = frc846::math::VectorND<units::feet_per_second_squared, 2>(
        corr_max_dcl,
        direction_offset + (corr_velocity > 0_fps ? -90_deg : 90_deg), true);
  } else {
    if (corr_velocity < CORRECTIONAL_GAIN * max_speed_ &&
        units::math::abs(dist_to_target_corr) > .5_in)
      corr_accl = frc846::math::VectorND<units::feet_per_second_squared, 2>(
          corr_max_accl,
          direction_offset + (dist_to_target_corr > 0_ft ? 90_deg : -90_deg),
          true);
    else
      corr_accl = {0_fps_sq, 0_fps_sq};
  }

  Graph("is_decelerating", is_decelerating_);
  Graph("stopping_distance", stopping_distance_dir);
  Graph("acceleration", (directional_accl + corr_accl).magnitude());

  dt_target.linear_acceleration = (directional_accl + corr_accl).magnitude();
  dt_target.accel_dir = (directional_accl + corr_accl).angle();

  dt_target.angular_velocity = drivetrain_->ApplyBearingPID(target_.bearing);

  drivetrain_->SetTarget(dt_target);
}

void DriveToPointCommand::End(bool interrupted) {
  Log("DriveToPointCommand ended with interruption status {} at position x: {} "
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

  if (drivetrain_readings.accel_vel <
          drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "accel_vel_stopped_thresh") ||
      drivetrain_readings.pose.velocity.magnitude() <
          drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "vel_stopped_thresh")) {
    num_stalled_loops_ += 1;
  } else {
    num_stalled_loops_ = 0;
  }

  return ((current_point - start_point_).magnitude() >=
             (target_.point - start_point_).magnitude()) ||
         (is_decelerating_ &&
             drivetrain_readings.pose.velocity.magnitude() <
                 drivetrain_
                     ->GetPreferenceValue_unit_type<units::feet_per_second_t>(
                         "vel_stopped_thresh"))

         || num_stalled_loops_ >
                drivetrain_->GetPreferenceValue_int("stopped_num_loops")

         || (end_when_close_ && (target_.point - current_point).magnitude() <
                                    3_in);  // TODO: prefify
}

}  // namespace frc846::robot::swerve