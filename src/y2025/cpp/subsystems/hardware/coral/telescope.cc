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
              .motor_current_limit = 90_A,
              .smart_current_limit = 120_A,
              .voltage_compensation = 12_V,
              .circuit_resistance =
                  robot_constants::coral_ss_::wire_resistance_base,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          262.5_in / 214.85_tr,
          robot_constants::telescope::telescope_hall_effect) {
  REGISTER_PIDF_CONFIG(0.025, 0.0, -0.001, 0.007);
  REGISTER_SOFTLIMIT_CONFIG(true, 90.5_in, 28.5_in, 78_in, 38.5_in, 0.3);
}

LinearSubsystemTarget TelescopeSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void TelescopeSubsystem::ExtendedSetup() {}