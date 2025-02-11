#include "subsystems/hardware/algal/algal_wrist.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

AlgalWristSubsystem::AlgalWristSubsystem()
    : WristSubsystem("algal_wrist",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::algal_ss_::wrist_::kWristMotor_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 40_A,
              .smart_current_limit = 30_A,
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0}},
          cancoder_reduction * cancoder_to_subsystem_reduction),
      cancoder_{ports::algal_ss_::wrist_::kWristCANCoder_CANID, "rio"} {}

WristTarget AlgalWristSubsystem::ZeroTarget() const {
  return WristTarget{0_deg};
}

void AlgalWristSubsystem::ExtendedSetup() {
  cancoder_.OptimizeBusUtilization();
  cancoder_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);
  RegisterPreference("use_sensor_threshold", 5_deg_per_s);
  RegisterPreference("cancoder_offset", 10_deg);
}

std::pair<units::degree_t, bool> AlgalWristSubsystem::GetSensorPos() {
  return {
      frc846::math::modulo(
          cancoder_.GetAbsolutePosition().GetValue() +
              GetPreferenceValue_unit_type<units::degree_t>("cancoder_offset"),
          360_deg) *
          (1 / cancoder_to_subsystem_reduction),
      units::math::abs(AlgalWristSubsystem::GetReadings().velocity) <
          GetPreferenceValue_unit_type<units::degrees_per_second_t>(
              "use_sensor_threshold")};
}