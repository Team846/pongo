#include "commands/teleop/drive_command.h"

#include <utility>

#include "calculators/gpd.h"

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

  // bool targeting_algae =
  // container_.control_input_.GetReadings().targeting_algae;

  //   std::vector<frc846::math::Vector2D> notes =
  //   container_.gpd_.GetReadings().notes;
  // run gpd
  std::vector<frc846::math::Vector2D> algae =
      std::vector<frc846::math::Vector2D>();
  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});
  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});
  units::second_t latency = gpdTable->GetNumber("tl", 0.1) * 1_s;

  std::pair<bool, frc846::math::Vector2D> gpd_calculations =
      gpd.calculate(algae, theta_x, container_.drivetrain_.GetReadings().pose,
          latency, distances);

  // if (targeting_algae) // add later && note is detected
  // { //turn towards the note
  //   frc846::math::Vector2D rel_algae_pos = gpd_calculations.second - //change
  //   to params from gpd method
  //                                         container_.drivetrain_.GetReadings().pose.point;
  //       drivetrain_target.rotation = DrivetrainRotationPosition(
  //           rel_algae_pos.Bearing());

  //       noteBearing =

  //                                   // field centric, negative because switch
  //                                   // between ccw to cs
  //   }
  //   container_.drivetrain_.SetTarget(drivetrain_target);
  //   }

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
  target.angular_velocity = rotation * max_omega;

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }