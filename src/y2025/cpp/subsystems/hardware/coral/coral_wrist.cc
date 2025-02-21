#include "subsystems/hardware/coral/coral_wrist.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

CoralWristSubsystem::CoralWristSubsystem()
    : WristSubsystem("coral_wrist",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::coral_ss_::wrist_::kWristMotor_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 20_A,
              .smart_current_limit = 10_A,
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::coral_ss_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          subsystem_reduction) {
  RegisterPreference("use_sensor_threshold", 5_deg_per_s);
  RegisterPreference("encoder_offset", 10_deg);
}

WristTarget CoralWristSubsystem::ZeroTarget() const {
  return WristTarget{0_deg};
}

void CoralWristSubsystem::ExtendedSetup() {}

std::pair<units::degree_t, bool> CoralWristSubsystem::GetSensorPos() {
  units::degree_t raw_enc_pos =
      CoralWristSubsystem::GetReadings().absolute_position;
  if (raw_enc_pos > 340_deg) raw_enc_pos -= 360_deg;
  return {raw_enc_pos +
              GetPreferenceValue_unit_type<units::degree_t>("encoder_offset"),
      units::math::abs(CoralWristSubsystem::GetReadings().velocity) <
          GetPreferenceValue_unit_type<units::degrees_per_second_t>(
              "use_sensor_threshold")};
}