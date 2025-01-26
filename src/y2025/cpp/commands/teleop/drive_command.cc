#include "commands/teleop/drive_command.h"
#include <utility>


DriveCommand::DriveCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});
}

void DriveCommand::OnInit() {}

void DriveCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  frc846::robot::swerve::DrivetrainOLControlTarget target{};

  bool targeting_algae = container_.control_input_.GetReadings().targeting_algae;
  
  if (targeting_algae) // add and a note is detected
  {
    // Turn towards the note
    frc846::util::Vector2D rel_note_pos =
        container_.gpd_.GPDCalculator.algae - 
        container_.drivetrain_.GetReadings().pose.point;
    error = container_. - container_.drivetrain_.GetReadings().pose.bearing
    drivetrain_target.rotation = GetPreferenceValue_double("gpd/kP") * error + GetPreferenceValue_double("gpd/kD") * velocity

  }
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
  target.angular_velocity = rotation * max_omega;

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }