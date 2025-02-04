#include "subsystems/hardware/algae_end_effector.h"

#include "subsystems/SubsystemHelper.h"

AlgaeEndEffectorSubsystem::AlgaeEndEffectorSubsystem()
    : frc846::robot::GenericSubsystem<AlgaeEndEffectorReadings,
          AlgaeEndEffectorTarget>("algae_end_effector"),
      motor_configs(GET_MOTOR_CONFIG("motor_configs", ports::ramp_::kRamp_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      algae_end_effector(
          frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX,
          motor_configs) {
  REGISTER_MOTOR_CONFIG("motor_configs", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("algae_end_effector_gains", 0.0, 0.0, 0.0, 0.0);

  motor_helper_.SetConversion(algae_pivot_reduction_);
  motor_helper_.bind(&algae_end_effector);
}

void AlgaeEndEffectorSubsystem::Setup() {
  algae_end_effector.Setup();
  algae_end_effector.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kVelocityFrame});
  algae_end_effector.ConfigForwardLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
  motor_helper_.SetPosition(0.0_ft);
}

AlgaeEndEffectorTarget AlgaeEndEffectorSubsystem::ZeroTarget() const {
  return AlgaeEndEffectorTarget{AlgaeEndEffectorState::kAlgaeIdle, 0.0_fps};
}

bool AlgaeEndEffectorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      algae_end_effector.VerifyConnected(), ok, "Could not verify ramp motor");
  return ok;
}

AlgaeEndEffectorReadings AlgaeEndEffectorSubsystem::ReadFromHardware() {
  AlgaeEndEffectorReadings readings;
  readings.vel = motor_helper_.GetVelocity();
  readings.gp_detected = algae_end_effector.GetForwardLimitSwitchState();
  Graph("readings/velocity", readings.vel);
  return readings;
}

void AlgaeEndEffectorSubsystem::WriteToHardware(AlgaeEndEffectorTarget target) {
  algae_end_effector.SetGains(GET_PIDF_GAINS("algae_end_effector_gains"));
  if (target.state == AlgaeEndEffectorState::kAlgaeScore) {
    target.vel = 2.0_fps;
  } else if (target.state == AlgaeEndEffectorState::kAlgaeIntake) {
    target.vel = 1.0_fps;
  } else {
    target.vel = 0.0_fps;
  }
  motor_helper_.WriteVelocity(target.vel);
}