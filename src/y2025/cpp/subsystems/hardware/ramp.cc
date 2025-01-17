#include "subsystems/hardware/ramp.h"

#include "subsystems/SubsystemHelper.h"

RampSubsystem::RampSubsystem()
    : frc846::robot::GenericSubsystem<RampReadings, RampTarget>("ramp"),
      motor_configs(GET_MOTOR_CONFIG("ramp/ramp_", ports::ramp_::kRamp_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      ramp_(frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX,
          motor_configs) {
  REGISTER_MOTOR_CONFIG("ramp/ramp_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("ramp/ramp_gains_", 0.0, 0.0, 0.0, 0.0);

  motor_helper_.SetConversion(ramp_reduction_);
  motor_helper_.bind(&ramp_);
}

void RampSubsystem::Setup() {
  ramp_.Setup();
  ramp_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kVelocityFrame});
}

RampTarget RampSubsystem::ZeroTarget() const {
  return RampTarget{RampState::kIdle, 0.0_fps};
}

bool RampSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(ramp_.VerifyConnected(), ok, "Could not verify ramp motor");
  return ok;
}

RampReadings RampSubsystem::ReadFromHardware() {
  RampReadings readings;
  readings.vel = motor_helper_.GetVelocity();
  Graph("readings/ramp_velocity", readings.vel);
  return readings;
}

void RampSubsystem::WriteToHardware(RampTarget target) {
  ramp_.SetGains(GET_PIDF_GAINS("ramp/ramp_gains_"));
  if (target.state == RampState::kIntake) {
    target.vel = 2.0_fps;
  } else {
    target.vel = 0.0_fps;
  }
  motor_helper_.WriteVelocity(target.vel);
}