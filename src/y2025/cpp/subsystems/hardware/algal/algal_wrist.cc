#include "subsystems/hardware/algal/algal_wrist.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

AlgalWristSubsystem::AlgalWristSubsystem()
    : WristSubsystem("algal_wrist",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::algal_ss_::wrist_::kWristMotor_CANID,
              .inverted = true,
              .brake_mode = true,
              .motor_current_limit = 50_A,
              .smart_current_limit = 40_A,
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0}},
          encoder_reduction * encoder_to_subsystem_reduction) {
  REGISTER_PIDF_CONFIG(0.005, 0.0, -0.0006, 0.0);
  REGISTER_SOFTLIMIT_CONFIG(true, 90_deg, 0_deg, 80_deg, 10_deg, 0.3);

  RegisterPreference("cg_offset", 90_deg);
  RegisterPreference("flip_position_load_sign", true);

  RegisterPreference("use_sensor_threshold", 5_deg_per_s);
  RegisterPreference("encoder_offset", 0_deg);
}

WristTarget AlgalWristSubsystem::ZeroTarget() const {
  return WristTarget{0_deg};
}

void AlgalWristSubsystem::ExtendedSetup() {}

std::pair<units::degree_t, bool> AlgalWristSubsystem::GetSensorPos(
    units::degree_t sensor_pos) {
  units::degree_t raw_enc_pos =
      AlgalWristSubsystem::GetReadings().absolute_position;
  if (raw_enc_pos > 120_deg) raw_enc_pos -= 360_deg;
  return {-raw_enc_pos * encoder_to_subsystem_reduction +
              GetPreferenceValue_unit_type<units::degree_t>("encoder_offset"),
      true/*units::math::abs(AlgalWristSubsystem::GetReadings().velocity) <
              GetPreferenceValue_unit_type<units::degrees_per_second_t>(
                  "use_sensor_threshold") &&
          GetReadings().position < 30_deg*/};
}