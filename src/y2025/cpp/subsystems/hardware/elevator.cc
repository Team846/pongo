#include "subsystems/hardware/elevator.h"

#include "subsystems/SubsystemHelper.h"

ElevatorSubsystem::ElevatorSubsystem()
    : frc846::robot::GenericSubsystem<ElevatorReadings, ElevatorTarget>(
          "elevator"),
      motor_configs(GET_MOTOR_CONFIG("elevator/elevator_one_",
          ports::elevator_::kElevatorOne_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      elevator_(frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60,
          motor_configs) {
  RegisterPreference("elevator_tolerance_", 0.25_in);

  REGISTER_MOTOR_CONFIG(
      "elevator/elevator_one_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("elevator/elevator_gains_", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("elevator/elevator_softlimits", true, 1.0);

  // bool using_limits =
  //     GetPreferenceValue_bool("elevator/elevator_softlimits/using_limits");
  // double reduce_max_dc =
  //     GetPreferenceValue_double("elevator/elevator_softlimits/reduce_max_dc");

  motor_helper_.SetConversion(elevator_reduction_);

  // motor_helper_.SetSoftLimits(
  //     using_limits, 96.0_in, 0.0_in, 90.0_in, 6.0_in, reduce_max_dc);

  motor_helper_.bind(&elevator_);
}

void ElevatorSubsystem::Setup() {
  elevator_.Setup();
  elevator_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});
  motor_helper_.SetPosition(0.0_in);
}

ElevatorTarget ElevatorSubsystem::ZeroTarget() const {
  return ElevatorTarget{0.0_in};
}

bool ElevatorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      elevator_.VerifyConnected(), ok, "Could not verify elevator motor");
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
  // motor_helper_.WritePosition(target.height);
  elevator_.SetLoad(1_Nm);
}