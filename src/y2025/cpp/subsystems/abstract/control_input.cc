#include "subsystems/abstract/control_input.h"

ControlInputSubsystem::ControlInputSubsystem(CoralSuperstructure* coral_ss)
    : frc846::robot::GenericSubsystem<ControlInputReadings,
          ControlInputTarget>{"control_input"},
      coral_ss_{coral_ss} {}

void ControlInputSubsystem::Setup() {
  RegisterPreference("trigger_threshold", 0.3);

  RegisterPreference("translation_deadband", 0.07);
  RegisterPreference("translation_exponent", 2);

  RegisterPreference("rotation_deadband", 0.07);
  RegisterPreference("rotation_exponent", 2);

  RegisterPreference("op_deadband", 0.35);
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
    base_adj = {0_in, 0_in};
  }
  if (readings.lock_right_reef != previous_readings_.lock_right_reef) {
    Log("ControlInput [Lock Right Reef] state changed to {}",
        readings.lock_right_reef ? 1 : 0);
    base_adj = {0_in, 0_in};
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

  ci_readings_.zero_bearing = dr_readings.back_button;
  ci_readings_.translate_x = dr_readings.left_stick_x;
  ci_readings_.translate_y = dr_readings.left_stick_y;

  ci_readings_.rc_p_y = (int)dr_readings.pov == 0;
  ci_readings_.rc_p_x = (int)dr_readings.pov == 90;
  // ci_readings_.rc_n_y = (int)dr_readings.pov == 180;
  ci_readings_.rc_n_x = (int)dr_readings.pov == 270;
  ci_readings_.rc_control = ci_readings_.rc_p_y || /*ci_readings_.rc_n_y ||*/
                            ci_readings_.rc_p_x || ci_readings_.rc_n_x;

  ci_readings_.rotation = dr_readings.right_stick_x;

  ci_readings_.lock_left_reef = dr_readings.left_bumper;
  ci_readings_.lock_right_reef = dr_readings.right_bumper;

  ci_readings_.targeting_algae = dr_readings.left_trigger;

  ci_readings_.auto_align = dr_readings.rsb;

  if (dr_readings.right_trigger && !previous_driver_.right_trigger)
    ci_readings_.position_algal = !previous_readings_.position_algal;
  else
    ci_readings_.position_algal = previous_readings_.position_algal;

  ci_readings_.override_autostow = op_readings.right_bumper;

  if ((coral_ss_->GetReadings().autostow_valid &&
          !ci_readings_.override_autostow) ||
      dr_readings.b_button)
    ci_readings_.coral_state = CoralStates::kCoral_StowNoPiece;
  else if (dr_readings.y_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL4;
  else if (dr_readings.x_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL3;
  else if (dr_readings.a_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL2;
  else
    ci_readings_.coral_state = previous_readings_.coral_state;

  if (op_readings.pov == frc846::robot::XboxPOV::kUp)
    ci_readings_.algal_state = AlgalStates::kAlgae_Net;
  else if (op_readings.pov == frc846::robot::XboxPOV::kRight)
    ci_readings_.algal_state = AlgalStates::kAlgae_L3Pick;
  else if (op_readings.pov == frc846::robot::XboxPOV::kDown)
    ci_readings_.algal_state = AlgalStates::kAlgae_Processor;
  else if (op_readings.pov == frc846::robot::XboxPOV::kLeft)
    ci_readings_.algal_state = AlgalStates::kAlgae_L2Pick;
  else if (op_readings.a_button)
    ci_readings_.algal_state = AlgalStates::kAlgae_GroundIntake;
  else if (op_readings.y_button)
    ci_readings_.algal_state = AlgalStates::kAlgae_OnTopIntake;
  else
    ci_readings_.algal_state = previous_readings_.algal_state;

  double op_deadband = GetPreferenceValue_double("op_deadband");

  if (op_readings.left_stick_y > op_deadband)
    ci_readings_.inc_telescope = true;
  else if (op_readings.left_stick_y < -op_deadband)
    ci_readings_.dec_telescope = true;
  if (op_readings.left_stick_x > op_deadband)
    ci_readings_.inc_c_wrist = true;
  else if (op_readings.left_stick_x < -op_deadband)
    ci_readings_.dec_c_wrist = true;

  if (op_readings.right_stick_y > op_deadband)
    ci_readings_.inc_elevator = true;
  else if (op_readings.right_stick_y < -op_deadband)
    ci_readings_.dec_elevator = true;
  if (op_readings.right_stick_x > op_deadband)
    ci_readings_.inc_a_wrist = true;
  else if (op_readings.right_stick_x < -op_deadband)
    ci_readings_.dec_a_wrist = true;

  ci_readings_.score_coral = op_readings.left_bumper;
  ci_readings_.score_algae = (dr_readings.pov == frc846::robot::XboxPOV::kDown);

  // TODO: add back climbing state
  // if (op_readings.left_trigger && !previous_operator_.left_trigger) {
  //   climb_state_ += 1;
  //   if (climb_state_ == 3) climb_state_ = 0;
  // }
  // ci_readings_.climb_state = climb_state_;

  Graph("climb_state", climb_state_);

  previous_driver_ = dr_readings;
  previous_operator_ = op_readings;

  return ci_readings_;
}