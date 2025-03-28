#include "subsystems/hardware/coral/coral_wrist.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

CoralWristSubsystem::CoralWristSubsystem()
    : WristSubsystem("coral_wrist",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::coral_ss_::wrist_::kWristMotor_CANID,
              .inverted = false,
              .brake_mode = false,
              .motor_current_limit = 80_A,
              .smart_current_limit = 60_A,
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::coral_ss_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          subsystem_reduction) {
  REGISTER_PIDF_CONFIG(0.0053, 0.0, -0.0005, 0.037);
  REGISTER_SOFTLIMIT_CONFIG(true, 260_deg, 5_deg, 230_deg, 15_deg, 0.15);

  RegisterPreference("cg_offset", -90.0_deg);
  RegisterPreference("flip_position_load_sign", false);

  RegisterPreference("use_sensor_threshold", 250_deg_per_s);
  RegisterPreference("encoder_offset", 0_deg);

  RegisterPreference("deployed_encoder_tolerance", 5_deg);
}

WristTarget CoralWristSubsystem::ZeroTarget() const {
  return WristTarget{0_deg};
}

void CoralWristSubsystem::ExtendedSetup() {
  units::degree_t raw_enc_pos =
      CoralWristSubsystem::GetReadings().absolute_position;
  if (raw_enc_pos > 340_deg) raw_enc_pos -= 360_deg;

  units::degree_t abs_pos =
      raw_enc_pos +
      GetPreferenceValue_unit_type<units::degree_t>("encoder_offset");

  wrist_esc_helper_.SetPosition(abs_pos);
}

// TODO: fix?
std::pair<units::degree_t, bool> CoralWristSubsystem::GetSensorPos(
    units::degree_t sensor_pos) {
  units::degree_t raw_enc_pos =
      sensor_pos;  // TODO: fix definitely wrong by at least one loop time
  if (raw_enc_pos > 340_deg) raw_enc_pos -= 360_deg;

  units::degree_t abs_pos =
      raw_enc_pos +
      GetPreferenceValue_unit_type<units::degree_t>("encoder_offset");

  bool is_valid_norm = true; /*
      units::math::abs(CoralWristSubsystem::GetReadings().velocity) <
      GetPreferenceValue_unit_type<units::degrees_per_second_t>(
          "use_sensor_threshold")*/ /*&&
  GetReadings().position < 50_deg*/

  //   bool is_valid_deployed =
  //       units::math::abs(CoralWristSubsystem::GetReadings().velocity) <
  //           GetPreferenceValue_unit_type<units::degrees_per_second_t>(
  //               "use_sensor_threshold") &&
  //       (GetReadings().position - abs_pos) >
  //           GetPreferenceValue_unit_type<units::degree_t>(
  //               "deployed_encoder_tolerance");

  return {abs_pos, is_valid_norm /*|| is_valid_deployed*/};
}