#include "subsystems/hardware/algal/algal_end_effector.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

AlgalEESubsystem::AlgalEESubsystem()
    : GenericSubsystem("algal_end_effector"),
      motor_configs_{
          .can_id = ports::algal_ss_::end_effector_::kEE1_CANID,
          .inverted = false,
          .brake_mode = false,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{0.0000062},
      },
      esc_1_{frc846::control::base::SPARK_MAX_NEO550,
          GetCurrentConfig(motor_configs_)},
      esc_2_{frc846::control::base::SPARK_MAX_NEO550,
          (GetCurrentConfig(GetModifiedConfig(motor_configs_,
              ports::algal_ss_::end_effector_::kEE2_CANID, true)))} {
  // RegisterPreference("idle_speed", 0.025);
  RegisterPreference("idle_speed", 2.3_fps);
  // RegisterPreference("piece_thresh", 2_tps);
  RegisterPreference("piece_thresh", 1.0_fps);

  // RegisterPreference("kick_dc", -0.2);
  RegisterPreference("kick_dc", -18.5_fps);
  // RegisterPreference("backspin_constant", -0.24);
  RegisterPreference("backspin_constant", -22_fps);
  RegisterPreference("idle_speed_coral", -5.0_fps);
  REGISTER_PIDF_CONFIG(0.0001, 0.0, 0.0, 0.0);

  RegisterPreference("testthing", 0.4);

  esc_helper_1_.SetConversion(roller_reduction_);
  esc_helper_2_.SetConversion(roller_reduction_);
  esc_helper_1_.bind(&esc_1_);
  esc_helper_2_.bind(&esc_2_);
}

frc846::control::config::MotorConstructionParameters
AlgalEESubsystem::GetCurrentConfig(
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

void AlgalEESubsystem::SetPieceOverride(bool override_piece) {
  piece_override_ = override_piece;
}

void AlgalEESubsystem::Setup() {
  esc_1_.Setup();
  esc_1_.EnableStatusFrames({});

  esc_2_.Setup();
  esc_2_.EnableStatusFrames({frc846::control::config::kFaultFrame});

  esc_2_.ConfigReverseLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
}

bool AlgalEESubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc 1");
  FRC846_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc 2");
  return ok;
}

AlgalEEReadings AlgalEESubsystem::ReadFromHardware() {
  AlgalEEReadings readings;

  readings.has_piece_ =
      esc_2_.GetReverseLimitSwitchState() &&
      units::math::abs(esc_helper_2_.GetVelocity()) <=
          GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "piece_thresh");

  if (piece_override_) readings.has_piece_ = false;

  Graph("readings/has_piece", readings.has_piece_);
  return readings;
}

void AlgalEESubsystem::WriteToHardware(AlgalEETarget target) {
  // Graph("target/duty_cycle", target.duty_cycle_);

  Graph("readings/error", target.velocity_ - esc_helper_2_.GetVelocity());

  Graph("readings/error_coral", target.velocity_ - esc_helper_1_.GetVelocity());

  esc_2_.SetGains(GET_PIDF_GAINS());

  // auto checkgains = frc846::control::base::MotorGains(GET_PIDF_GAINS());

  if (GetReadings().has_piece_ && target.velocity_ > 0.0_fps) {
   
      target.velocity_ =
        GetPreferenceValue_unit_type<units::feet_per_second_t>("idle_speed");
    
  }
  if (target.cm) {
    // if (counter_ > 5) {
    // target.velocity_ = GetPreferenceValue_unit_type<units::feet_per_second_t>("idle_speed_coral");
    // if(counter_ >= 10) counter_ = 0;
    // } else {
    //   target.velocity_ = 0_fps;

    // }
    // counter_++;
    target.velocity_ = -5.0_fps;
  }

  if (units::math::abs(esc_helper_2_.GetVelocity()) <=
          GetPreferenceValue_unit_type<units::feet_per_second_t>(
              "piece_thresh") &&
      target.velocity_ < 0.0_fps) {
    target.velocity_ =
        GetPreferenceValue_unit_type<units::feet_per_second_t>("kick_dc");
  }

  if (piece_override_) { target.velocity_ = 0.0_fps; }
  if (target.use_back_spin_) {
    esc_helper_1_.WriteVelocityOnController(
        target.velocity_ +
        GetPreferenceValue_unit_type<units::feet_per_second_t>(
            "backspin_constant"));
    esc_helper_2_.WriteVelocityOnController(
        target.velocity_ -
        GetPreferenceValue_unit_type<units::feet_per_second_t>(
            "backspin_constant"));
  } else {
    esc_helper_1_.WriteVelocityOnController(target.velocity_);
    esc_helper_2_.WriteVelocityOnController(target.velocity_);
  }
  // esc_helper_1_.WriteDC((target.velocity_/80_fps).to<double>());
  // esc_helper_2_.WriteDC((target.velocity_/80_fps).to<double>());
}