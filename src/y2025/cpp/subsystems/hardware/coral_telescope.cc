#include "subsystems/hardware/coral_telescope.h"

#include "subsystems/SubsystemHelper.h"

TelescopeSubsystem::TelescopeSubsystem()
    : frc846::robot::GenericSubsystem<TelescopeReadings, TelescopeTarget>(
          "coral_telescope"),
      motor_configs(GET_MOTOR_CONFIG("motor_configs",
          ports::telescope_::kTelescopeOne_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      telescope_(frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX,
          motor_configs) {
  RegisterPreference("coral_telescope_tolerance", 0.25_in);

  REGISTER_MOTOR_CONFIG("motor_configs", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("coral_telescope_gains", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG(
      "coral_telescope_limits", true, 72_in, 30_in, 66_in, 34_in, 0.3);

  motor_helper_.SetConversion(0.5_in / 1.0_tr);
  motor_helper_.SetSoftLimits(
      GET_SOFTLIMITS("coral_telescope_limits", units::inch_t));
  motor_helper_.SetControllerSoftLimits(
      GET_SOFTLIMITS("coral_telescope_limits", units::inch_t));

  motor_helper_.bind(&telescope_);
}

void TelescopeSubsystem::Setup() {
  telescope_.Setup();
  telescope_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kVelocityFrame});
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  return TelescopeTarget{0.0_in};
}

bool TelescopeSubsystem::VerifyHardware() {
  bool ok = true;

  FRC846_VERIFY(
      telescope_.VerifyConnected(), ok, "Telescope ESC not connected");

  return ok;
}

TelescopeReadings TelescopeSubsystem::ReadFromHardware() {
  TelescopeReadings readings;

  readings.position = motor_helper_.GetPosition();

  Graph("readings/position", readings.position);

  return readings;
}

void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  telescope_.SetGains(GET_PIDF_GAINS("coral_telescope/coral_telescope_gains_"));
  motor_helper_.WritePosition(target.position);

  telescope_.SetLoad(1_Nm);
}
