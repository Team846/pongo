#include "subsystems/abstract/control_input.h"

#include "field.h"
#include "reef.h"

ControlInputSubsystem::ControlInputSubsystem(CoralSuperstructure* coral_ss,
    AlgalSuperstructure* algal_ss,
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain_ss)
    : frc846::robot::GenericSubsystem<ControlInputReadings,
          ControlInputTarget>{"control_input"},
      coral_ss_{coral_ss},
      algal_ss_{algal_ss},
      drivetrain_ss_{drivetrain_ss} {}

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
  if (frc::DriverStation::IsDisabled()) { first_enable_exception = true; }
  if (frc::DriverStation::IsAutonomous()) { first_enable_exception = false; }

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

  if (dr_readings.right_trigger && !previous_driver_.right_trigger) {
    ci_readings_.position_algal = !previous_readings_.position_algal;
    if (!ci_readings_.position_algal) { op_changed_target_ = false; }
  } else
    ci_readings_.position_algal = previous_readings_.position_algal;

  if (frc::DriverStation::IsDisabled()) ci_readings_.position_algal = false;

  ci_readings_.override_autostow = op_readings.right_bumper;

  bool previous_first_enable_exception = first_enable_exception;

  first_enable_exception = false;
  if (frc::DriverStation::IsDisabled() ||
      (coral_ss_->GetReadings().autostow_valid &&
          !ci_readings_.override_autostow) ||
      dr_readings.b_button)
    ci_readings_.coral_state = CoralStates::kCoral_StowNoPiece;
  else if (dr_readings.y_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL4;
  else if (dr_readings.x_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL3;
  else if (dr_readings.a_button)
    ci_readings_.coral_state = CoralStates::kCoral_ScoreL2;
  else {
    ci_readings_.coral_state = previous_readings_.coral_state;
    first_enable_exception = previous_first_enable_exception;
  }

  previous_first_enable_exception = first_enable_exception;

  // algae autopicking

  AlgalStates previous_state = previous_readings_.algal_state;

  bool operator_clicked = false;
  first_enable_exception = false;
  if (op_readings.pov == frc846::robot::XboxPOV::kUp) {
    ci_readings_.algal_state = AlgalStates::kAlgae_Net;
    operator_clicked = true;

  } else if (op_readings.pov == frc846::robot::XboxPOV::kRight) {
    ci_readings_.algal_state = AlgalStates::kAlgae_L3Pick;

    operator_clicked = true;
  } else if (op_readings.pov == frc846::robot::XboxPOV::kDown) {
    ci_readings_.algal_state = AlgalStates::kAlgae_Processor;

    operator_clicked = true;
  } else if (op_readings.pov == frc846::robot::XboxPOV::kLeft) {
    ci_readings_.algal_state = AlgalStates::kAlgae_L2Pick;

    operator_clicked = true;
  } else if (op_readings.a_button) {
    ci_readings_.algal_state = AlgalStates::kAlgae_GroundIntake;

    operator_clicked = true;
  } else if (op_readings.y_button) {
    ci_readings_.algal_state = AlgalStates::kAlgae_OnTopIntake;

    operator_clicked = true;
  } else {
    ci_readings_.algal_state = previous_readings_.algal_state;
    first_enable_exception = previous_first_enable_exception;
  }

  if (ci_readings_.algal_state != previous_state && operator_clicked) {
    op_changed_target_ = true;
  }

  auto drivetrain_readings = drivetrain_ss_->GetReadings();
  auto curr_pose = drivetrain_readings.estimated_pose.position;
  auto rotation = drivetrain_readings.estimated_pose.bearing;

  // autopicking for l2/l3
  if (ci_readings_.lock_left_reef) {
    ci_readings_.algal_state =
        (ReefProvider::getClosestReefSide(curr_pose) % 2 == 0)
            ? AlgalStates::kAlgae_L2Pick
            : AlgalStates::kAlgae_L3Pick;
  }

  units::inch_t mid_field_y = frc846::math::FieldPoint::field_size_y / 2.0;

  if (algal_ss_->GetReadings().has_piece) {
    no_algae_counter = 0;
  } else {
    if (no_algae_counter < 1000) no_algae_counter++;
  }

  // Net autopicking
  //  check if y is within 80 in of midfield
  if (units::math::abs(curr_pose[1] - mid_field_y) < 80_in &&
      !op_changed_target_ && algal_ss_->GetReadings().has_piece) {
    // check if robot is pointed within 30 deg of 0 or 180 deg
    if (units::math::abs(rotation) < 30_deg ||
        units::math::abs(rotation - 180_deg) < 30_deg) {
          // Log("net auto");
      ci_readings_.algal_state = AlgalStates::kAlgae_Net;
    }
  }

  units::inch_t field_width = frc846::math::FieldPoint::field_size_x;

  // processor autopicking
  //  check if near right side and pointed at 90 degree and 30 in from the right
  if (units::math::abs(field_width - curr_pose[0]) < 30.0_in &&
      units::math::abs(rotation - 90.0_deg) < 30.0_deg && !op_changed_target_ &&
      algal_ss_->GetReadings().has_piece){
        // Log("processor auto");
    ci_readings_.algal_state = AlgalStates::kAlgae_Processor;
  }
  // check if near left side and pointed at -90 degree and 30 in from the left
  else if (curr_pose[0] < 30.0_in &&
           units::math::abs(rotation + 90.0_deg) < 30.0_deg &&
           !op_changed_target_ && algal_ss_->GetReadings().has_piece) {
    // Log("proc auto");
    ci_readings_.algal_state = AlgalStates::kAlgae_Processor;
  }

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

  if (ci_readings_.score_algae || ci_readings_.score_coral)
    first_enable_exception = false;

  ci_readings_.extend_climb = op_readings.right_trigger;
  ci_readings_.retract_climb = op_readings.left_trigger;

  ci_readings_.override_soft_limits = op_readings.back_button;
  ci_readings_.home_telescope =
      op_readings.x_button && ci_readings_.override_soft_limits;
  ci_readings_.home_elevator =
      op_readings.b_button && ci_readings_.override_soft_limits;

  ci_readings_.flick = op_readings.lsb;

  previous_driver_ = dr_readings;
  previous_operator_ = op_readings;

  ci_readings_.first_enable_exception = first_enable_exception;
  Graph("first_enable_exception", first_enable_exception);

  return ci_readings_;
}