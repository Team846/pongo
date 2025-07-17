#include "subsystems/hardware/generic/linear_subsystem.h"

#include "subsystems/SubsystemHelper.h"

LinearSubsystem::LinearSubsystem(std::string name,
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters motor_configs_,
    linear_pos_conv_t conversion, units::inch_t hall_effect_loc_)
    : frc846::robot::GenericSubsystem<LinearSubsystemReadings,
          LinearSubsystemTarget>(name),
      linear_esc_(mmtype, GetCurrentConfig(motor_configs_)),
      hall_effect_loc_(hall_effect_loc_),
      homing_state_(HomingState::kNotHoming),
      home_loop_counter_(0),
      homing_target_(0_in) {
  linear_esc_helper_.SetConversion(conversion);
  linear_esc_helper_.bind(&linear_esc_);
  RegisterPreference("pidf_deadband", 0.35_in);
  RegisterPreference("homing_dc", -0.15);
  RegisterPreference("homing_thresh", 0.005_fps);
  RegisterPreference("homing_loops", 25);
  // RegisterPreference("home_zero_height", 28.0_in); 
  // RegisterPreference("telescope_autohome_height", 27.0_in);
}

frc846::control::config::MotorConstructionParameters
LinearSubsystem::GetCurrentConfig(
    frc846::control::config::MotorConstructionParameters original_config) {
  frc846::control::config::MotorConstructionParameters modifiedConfig =
      original_config;
  REGISTER_MOTOR_CONFIG(
      original_config.motor_current_limit, original_config.smart_current_limit);
  modifiedConfig.motor_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/current_limit");
  modifiedConfig.smart_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/smart_current_limit");
  return modifiedConfig;
}

void LinearSubsystem::Setup() {
  linear_esc_.Setup();

  linear_esc_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame,
          frc846::control::config::StatusFrame::kFaultFrame});

  linear_esc_helper_.SetPosition(28.5_in);

  linear_esc_helper_.SetSoftLimits(GET_SOFTLIMITS(units::inch_t));
  // linear_esc_helper_.SetControllerSoftLimits(GET_SOFTLIMITS(units::inch_t));

  // linear_esc_.ConfigForwardLimitSwitch(
  //     false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);

  ExtendedSetup();
}

void LinearSubsystem::HomeSubsystem(units::inch_t pos) {
  if (is_initialized()) linear_esc_helper_.SetPosition(pos);
  is_homed_ = true;
}

bool LinearSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(linear_esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

LinearSubsystemReadings LinearSubsystem::ReadFromHardware() {
  LinearSubsystemReadings readings;
  readings.position = linear_esc_helper_.GetPosition();

  linear_esc_.SetLoad(1_Nm);

  Graph("readings/position", readings.position);
  Graph("readings/current_draw", linear_esc_.GetCurrent());

  Graph("readings/error", GetTarget().position - readings.position);

  Graph("readings/homing_dc",GetPreferenceValue_double("homing_dc"));
  Graph("readings/homing_thresh",
        GetPreferenceValue_unit_type<units::feet_per_second_t>("homing_thresh"));
  Graph("readings/loops", GetPreferenceValue_int("homing_loops"));

  RHExtension();

  UpdateHoming();

  // bool forward_limit = linear_esc_.GetForwardLimitSwitchState();

  // Graph("readings/homing_sensor", forward_limit);

  // if (forward_limit && !is_homed_) {
  //   is_homed_ = true;
  //   linear_esc_helper_.SetPosition(hall_effect_loc_);
  // }

  return readings;
}

void LinearSubsystem::OverrideSoftLimits(bool overrideLimits) {
  frc846::control::SoftLimitsConfig limits = GET_SOFTLIMITS(units::inch_t);

  limits.using_limits = !overrideLimits;
  linear_esc_helper_.SetSoftLimits(limits);
}

void LinearSubsystem::WriteToHardware(LinearSubsystemTarget target) {
  Graph("target/position", target.position);
  linear_esc_.SetGains(GET_PIDF_GAINS());

  linear_esc_.SetLoad(1_Nm);

  if (GetReadings().position > GetPreferenceValue_unit_type<units::inch_t>(
                                   "telescopel4_modifier_height")) {
    linear_esc_.SetLoad(GetPreferenceValue_unit_type<units::newton_meter_t>(
        "telescopel4_load"));
  }

  units::inch_t deadband =
      GetPreferenceValue_unit_type<units::inch_t>("pidf_deadband");

  if (units::math::abs(GetReadings().position - target.position) >
      (deadband * 1.2))
    exit_deadband = true;

  if (units::math::abs(GetReadings().position - target.position) <=
      (deadband * 0.6))
    exit_deadband = false;

  if (exit_deadband) {
    Graph("within_deadband", false);
    linear_esc_helper_.WritePosition(target.position);
  } else {
    Graph("within_deadband", true);
    linear_esc_helper_.WriteDC(0.0);
  }
}

void LinearSubsystem::BrakeSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(true);
}

void LinearSubsystem::CoastSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(false);
}

void LinearSubsystem::StartHoming(units::inch_t target) {
    homing_target_ = target;
    homing_state_ = HomingState::kHoming;
    home_loop_counter_ = 0;
    is_homed_ = false;
}

void LinearSubsystem::UpdateHoming() {
    if (homing_state_ == HomingState::kHoming) {
        double homing_dc = GetPreferenceValue_double("homing_dc");
        linear_esc_helper_.WriteDC(homing_dc);

        units::feet_per_second_t velocity_threshold =
            GetPreferenceValue_unit_type<units::feet_per_second_t>("homing_thresh");

        if (units::math::abs(linear_esc_helper_.GetVelocity()) < velocity_threshold) {
            home_loop_counter_++;
        } else {
            home_loop_counter_ = 0;
        }

        int required_loops = GetPreferenceValue_int("homing_loops");
        if (home_loop_counter_ >= required_loops) {
            linear_esc_helper_.SetPosition(homing_target_);
            SetTarget({homing_target_});
            homing_state_ = HomingState::kHomingComplete;
            home_loop_counter_ = 0;
            is_homed_ = true;
        }
    }
}
