#include "subsystems/hardware/algae_pivot.h"

#include "subsystems/SubsystemHelper.h"

AlgaePivotSubsystem::AlgaePivotSubsystem()
    : frc846::robot::GenericSubsystem<AlgaePivotReadings, AlgaePivotTarget>(
          "algae_pivot"),
      motor_configs(GET_MOTOR_CONFIG("algae_pivot/algae_pivot_one_",
          ports::algae_pivot_::kAlgaePivotOne_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      pivot_(frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX,
          motor_configs) {
  RegisterPreference("algae_pivot_tolerance_", 0.25_deg);

  REGISTER_MOTOR_CONFIG(
      "algae_pivot/algae_pivot_one_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("algae_pivot/algae_pivot_gains_", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("algae_pivot/algae_pivot_softlimits", true, 1.0);

  // bool using_limits =
  // GetPreferenceValue_bool("algae_pivot/algae_pivot_softlimits/using_limits");
  // double reduce_max_dc =
  // GetPreferenceValue_double("algae_pivot/algae_pivot_softlimits/reduce_max_dc");

  motor_helper_.SetConversion(algae_pivot_reduction_);

  arm_calculator_.setConstants({20.0_kg, 3.0_in, 0.0_deg, -1.0, 1.0});

  // motor_helper_.SetSoftLimits(
  //     using_limits, 90_deg, 0.0_deg, 80_deg, 5_deg, reduce_max_dc);

  motor_helper_.bind(&pivot_);
}

void AlgaePivotSubsystem::Setup() {
  pivot_.Setup();

  pivot_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});

  motor_helper_.SetPosition(0.0_deg);
}

AlgaePivotTarget AlgaePivotSubsystem::ZeroTarget() const {
  return AlgaePivotTarget{0.0_deg};
}

bool AlgaePivotSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      pivot_.VerifyConnected(), ok, "Could not verify algae pivot motor");
  return ok;
}

AlgaePivotReadings AlgaePivotSubsystem::ReadFromHardware() {
  AlgaePivotReadings readings;
  readings.position = motor_helper_.GetPosition();
  readings.gp_detected = pivot_.GetForwardLimitSwitchState();

  Graph("readings/position", readings.position);
  Graph("readings/game_piece_detected", readings.gp_detected);
  return readings;
}

void AlgaePivotSubsystem::WriteToHardware(AlgaePivotTarget target) {
  pivot_.SetGains(GET_PIDF_GAINS("algae_pivot/algae_pivot_gains_"));
  motor_helper_.WriteDC(arm_calculator_.calculate({motor_helper_.GetPosition(),
      target.position, motor_helper_.GetVelocity(),
      GET_PIDF_GAINS("algae_pivot/algae_pivot_gains_")}));
}