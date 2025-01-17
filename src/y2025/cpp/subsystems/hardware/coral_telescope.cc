#include "subsystems/hardware/coral_telescope.h"

#include "subsystems/SubsystemHelper.h"

TelescopeSubsystem::TelescopeSubsystem()
    : frc846::robot::GenericSubsystem<TelescopeReadings, TelescopeTarget>(
          "telescope"),
      motor_configs(GET_MOTOR_CONFIG("telescope/telescope_one_",
          ports::telescope_::kTelescopeOne_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      telescope_(frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60,
          motor_configs) {
  RegisterPreference("telescope/telescope_tolerance_", 0.25_in);

  REGISTER_MOTOR_CONFIG(
      "telescope/telescope_one_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("telescope/telescope_gains_", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("telescope/telescope_softlimits", true, 1.0);

  motor_helper_.SetConversion(telescope_reduction);

  motor_helper_.bind(&telescope_);
}

void TelescopeSubsystem::Setup() { telescope_.Setup(); }

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  TelescopeTarget target;
  target.extension = 0.0_in;
  return target;
}

bool TelescopeSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(
        telescope_.VerifyConnected(), ok, "Telescope ESC not connected");

    return ok;
  }
  return true;
}

TelescopeReadings TelescopeSubsystem::ReadFromHardware() {
  TelescopeReadings readings;
  readings.extension = motor_helper_.GetPosition();

  Graph("Telescope extension", readings.extension);

  return readings;
}

void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  telescope_.SetGains(GET_PIDF_GAINS("telescope/telescope_gains_"));
  motor_helper_.WritePosition(target.extension);
}
