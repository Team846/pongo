#include "subsystems/abstract/gpd.h"

//#include "subsystems/SubsystemHelper.h"
  
GPDSubsystem::GPDSubsystem()
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>(
          "gpd"),
      motor_configs(GET_MOTOR_CONFIG("gpd/gpd_one_",
          ports::gpd_::kGPDOne_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      gpd_(frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60,
          motor_configs) {
  RegisterPreference("gpd_tolerance_", 0.25_in);

  
}

void GPDSubsystem::Setup() {
  gpd_.Setup();
  gpd.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});
  motor_helper_.SetPosition(0.0_in);
}

GPDTarget GPDSubsystem::ZeroTarget() const {
  return GPDTarget{0.0_in};
}

bool GPDSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      gpd_.VerifyConnected(), ok, "Could not verify gpd motor");
  return ok;
}

ElevatorReadings ElevatorSubsystem::ReadFromHardware() {
  ElevatorReadings readings;
  readings.height = motor_helper_.GetPosition();
  Graph("readings/elevator_position", readings.height);
  return readings;
}

void ElevatorSubsystem::WriteToHardware(ElevatorTarget target) {
  elevator_.SetGains(GET_PIDF_GAINS("elevator/elevator_gains_"));
  motor_helper_.WritePosition(target.height);
}