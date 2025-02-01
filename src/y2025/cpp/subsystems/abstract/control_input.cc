#include "subsystems/abstract/control_input.h"

ControlInputSubsystem::ControlInputSubsystem()
    : frc846::robot::GenericSubsystem<ControlInputReadings, ControlInputTarget>{
          "control_input"} {}

void ControlInputSubsystem::Setup() {
  RegisterPreference("trigger_threshold", 0.3);

  RegisterPreference("translation_deadband", 0.07);
  RegisterPreference("translation_exponent", 2);

  RegisterPreference("rotation_deadband", 0.07);
  RegisterPreference("rotation_exponent", 2);
}

ControlInputTarget ControlInputSubsystem::ZeroTarget() const {
  ControlInputTarget target;
  target.driver_rumble = false;
  target.operator_rumble = false;
  return target;
}

bool ControlInputSubsystem::VerifyHardware() { return true; }

ControlInputReadings ControlInputSubsystem::ReadFromHardware() {
  ControlInputReadings readings = UpdateWithInput();

  if (readings.zero_bearing != previous_readings_.zero_bearing) {
    Log("ControlInput [Drivetrain Zeroing] state changed to {}",
        readings.zero_bearing ? 1 : 0);
  }

  if (readings.lock_left_reef != previous_readings_.lock_left_reef) {
    Log("ControlInput [Lock Left Reef] state changed to {}",
        readings.lock_left_reef ? 1 : 0);
  }
  if (readings.lock_right_reef != previous_readings_.lock_right_reef) {
    Log("ControlInput [Lock Right Reef] state changed to {}",
        readings.lock_right_reef ? 1 : 0);
  }

  previous_readings_ = readings;

  return readings;
}

void ControlInputSubsystem::WriteToHardware(ControlInputTarget target) {
  driver_.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
      target.driver_rumble ? 1.0 : 0.0);
  operator_.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
      target.driver_rumble ? 1.0 : 0.0);
}

ControlInputReadings ControlInputSubsystem::UpdateWithInput() {
  ControlInputReadings ci_readings_{};
  auto trigger_threshold = GetPreferenceValue_double("trigger_threshold");
  frc846::robot::XboxReadings dr_readings{driver_, trigger_threshold};
  frc846::robot::XboxReadings op_readings{operator_, trigger_threshold};

  // DRIVETRAIN
  ci_readings_.zero_bearing = dr_readings.back_button;
  ci_readings_.translate_x = dr_readings.left_stick_x;
  ci_readings_.translate_y = dr_readings.left_stick_y;

  ci_readings_.rc_p_y = (int)dr_readings.pov == 0;
  ci_readings_.rc_p_x = (int)dr_readings.pov == 90;
  ci_readings_.rc_n_y = (int)dr_readings.pov == 180;
  ci_readings_.rc_n_x = (int)dr_readings.pov == 270;
  ci_readings_.rc_control = ci_readings_.rc_p_y || ci_readings_.rc_n_y ||
                            ci_readings_.rc_p_x || ci_readings_.rc_n_x;

  ci_readings_.rotation = dr_readings.right_stick_x;

  ci_readings_.test_move_10_ft = dr_readings.x_button;
  ci_readings_.test_bearing_pid = dr_readings.y_button;

  ci_readings_.lock_left_reef = dr_readings.left_bumper;
  ci_readings_.lock_right_reef = dr_readings.right_bumper;

  ci_readings_.targeting_algae = dr_readings.left_trigger;

  return ci_readings_;
}