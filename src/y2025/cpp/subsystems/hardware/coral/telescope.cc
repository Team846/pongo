#include "subsystems/hardware/coral/telescope.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

TelescopeSubsystem::TelescopeSubsystem()
    : LinearSubsystem("telescope",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::coral_ss_::telescope_::kTelescope_CANID,
              .inverted = true,
              .brake_mode = true,
              .motor_current_limit = 60_A,
              .smart_current_limit = 120_A,
              .voltage_compensation = 12_V,
              .circuit_resistance =
                  robot_constants::coral_ss_::wire_resistance_base,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{0.0005}},
          262.5_in / 214.85_tr * 9.0 / 5.0,
          robot_constants::telescope::telescope_hall_effect) {
  REGISTER_PIDF_CONFIG(0.026, 0.0, -0.0015, 0.0295);
  REGISTER_SOFTLIMIT_CONFIG(true, 91.25_in, 28.5_in, 87_in, 40_in, 0.45);
  RegisterPreference("telescopel4_modifier_height", 70_in);
  RegisterPreference("telescopel4_load", 2.1_Nm);
  RegisterPreference("telescope_hysteresis", true);
}

LinearSubsystemTarget TelescopeSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void TelescopeSubsystem::ExtendedSetup() {}