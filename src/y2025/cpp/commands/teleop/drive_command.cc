#include "commands/teleop/drive_command.h"

#include <utility>

#include "calculators/AntiTippingCalculator.h"
#include "frc846/math/fieldpoints.h"

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
      ci_readings_.translate_x,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double translate_y = frc846::math::HorizontalDeadband(
      ci_readings_.translate_y,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double rotation = frc846::math::HorizontalDeadband(ci_readings_.rotation,
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

  if (ci_readings_.rc_control) {
    frc846::math::VectorND<units::feet_per_second, 2> vel_rc{0_fps, 0_fps};

    units::feet_per_second_t rc_speed =
        container_.drivetrain_
            .GetPreferenceValue_unit_type<units::feet_per_second_t>(
                "rc_control_speed");

    if (ci_readings_.rc_p_x) {
      vel_rc[0] = rc_speed;
      vel_rc[1] = 0_fps;
    } else if (ci_readings_.rc_n_x) {
      vel_rc[0] = -rc_speed;
      vel_rc[1] = 0_fps;
    } else if (ci_readings_.rc_p_y) {
      vel_rc[0] = 0_fps;
      vel_rc[1] = rc_speed;
    } else if (ci_readings_.rc_n_y) {
      vel_rc[0] = 0_fps;
      vel_rc[1] = -rc_speed;
    }

    target.velocity =
        vel_rc.rotate(container_.drivetrain_.GetReadings().pose.bearing, true);
  }

  target.angular_velocity = rotation * max_omega;

  bool isBlue = (frc::DriverStation::GetAlliance() ==
                 frc::DriverStation::Alliance::kBlue);

  if (isBlue) target.velocity = target.velocity.rotate(180_deg);

  if (ci_readings_.auto_align) {
    units::degree_t target_angle = 1000_deg;
    if (ci_readings_.algal_state == kAlgae_Net &&
        container_.algal_ss_.algal_end_effector.GetReadings().has_piece_) {
      target_angle = isBlue ? 180_deg : 0_deg;
    } else if (ci_readings_.algal_state == kAlgae_Processor &&
               container_.algal_ss_.algal_end_effector.GetReadings()
                   .has_piece_) {
      target_angle = isBlue ? -90_deg : 90_deg;
    } else if (ci_readings_.coral_state == kCoral_StowNoPiece &&
               !container_.coral_ss_.coral_end_effector.GetReadings()
                    .has_piece_) {
      if (container_.drivetrain_.GetReadings().estimated_pose.position[0] >
          (frc846::math::FieldPoint::field_size_x / 2))
        target_angle = -54_deg;
      else
        target_angle = 54_deg;

      if (isBlue) target_angle = 180_deg - target_angle;
    }
    if (target_angle <= 720_deg)
      target.angular_velocity =
          container_.drivetrain_.ApplyBearingPID(target_angle);
  }

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }