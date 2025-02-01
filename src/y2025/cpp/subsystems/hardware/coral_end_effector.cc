#include "subsystems/hardware/coral_end_effector.h"

#include "subsystems/SubsystemHelper.h"

CoralEndEffectorSubsystem::CoralEndEffectorSubsystem()
    : frc846::robot::GenericSubsystem<CoralEndEffectorReadings,
          CoralEndEffectorTarget>("coral_end_effector"),
      motor_configs(GET_MOTOR_CONFIG("motor_configs", ports::ramp_::kRamp_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      coral_end_effector(
          frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX,
          motor_configs) {
  REGISTER_MOTOR_CONFIG("motor_configs", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("coral_end_effector_gains", 0.0, 0.0, 0.0, 0.0);

  motor_helper_.SetConversion(coral_pivot_reduction_);
  motor_helper_.bind(&coral_end_effector);
}

void CoralEndEffectorSubsystem::Setup() {
  coral_end_effector.Setup();
  coral_end_effector.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kVelocityFrame});
  coral_end_effector.ConfigForwardLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
  motor_helper_.SetPosition(0.0_ft);
}

CoralEndEffectorTarget CoralEndEffectorSubsystem::ZeroTarget() const {
  return CoralEndEffectorTarget{CoralEndEffectorState::kIdle, 0.0_fps};
}

bool CoralEndEffectorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      coral_end_effector.VerifyConnected(), ok, "Could not verify ramp motor");
  return ok;
}

CoralEndEffectorReadings CoralEndEffectorSubsystem::ReadFromHardware() {
  CoralEndEffectorReadings readings;
  readings.vel = motor_helper_.GetVelocity();
  readings.reef_detected = coral_end_effector.GetForwardLimitSwitchState();
  Graph("readings/velocity", readings.vel);
  Graph("readings/reef_detection", readings.reef_detected);
  return readings;
}

void CoralEndEffectorSubsystem::WriteToHardware(CoralEndEffectorTarget target) {
  coral_end_effector.SetGains(GET_PIDF_GAINS("coral_end_effector_gains"));
  if (target.state == CoralEndEffectorState::kScore) {
    target.vel = 2.0_fps;
  } else if (target.state == CoralEndEffectorState::kIntake) {
    target.vel = 1.0_fps;
  } else {
    target.vel = 0.0_fps;
  }
  motor_helper_.WriteVelocity(target.vel);
}