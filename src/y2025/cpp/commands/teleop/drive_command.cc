#include "commands/teleop/drive_command.h"

#include "calculators/AntiTippingCalculator.h"

DriveCommand::DriveCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});
}

void DriveCommand::OnInit() {}

void DriveCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  frc846::robot::swerve::DrivetrainOLControlTarget target{};

  container_.drivetrain_.SetTarget({target});

  double translate_x = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().translate_x,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double translate_y = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().translate_y,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double rotation = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().rotation,
      container_.control_input_.GetPreferenceValue_double("rotation_deadband"),
      1, container_.control_input_.GetPreferenceValue_int("rotation_exponent"),
      1);

  units::feet_per_second_t max_speed =
      container_.drivetrain_
          .GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed");
  units::degrees_per_second_t max_omega =
      container_.drivetrain_
          .GetPreferenceValue_unit_type<units::degrees_per_second_t>(
              "max_omega");

  target.velocity = {translate_x * max_speed, translate_y * max_speed};

  auto delta_dir = (frc846::math::VectorND<units::feet_per_second, 2>{
                        target.velocity[0], target.velocity[1]} -
                    container_.drivetrain_.GetReadings().pose.velocity);

  Graph("delta_dir_x", delta_dir[0]);
  Graph("delta_dir_y", delta_dir[1]);

  auto accel_limited = AntiTippingCalculator::LimitAcceleration(
      delta_dir, container_.drivetrain_.GetReadings().pose.bearing);

  Graph("limited_accel_x", accel_limited[0]);
  Graph("limited_accel_y", accel_limited[1]);

  target.velocity[0] =
      1_fps * rampRateLimiter_x_.limit(target.velocity[0].to<double>(),
                  accel_limited[0].to<double>());
  target.velocity[1] =
      1_fps * rampRateLimiter_y_.limit(target.velocity[1].to<double>(),
                  accel_limited[1].to<double>());

  target.angular_velocity = rotation * max_omega;

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }