#include "frc846/robot/swerve/swerve_module.h"

namespace frc846::robot::swerve {

SwerveModule::SwerveModule(Loggable& parent, std::string loc,
    frc846::control::config::MotorConstructionParameters drive_params,
    frc846::control::config::MotorConstructionParameters steer_params,
    frc846::control::base::MotorMonkeyType motor_types, int cancoder_id,
    steer_conv_unit steer_reduction, drive_conv_unit drive_reduction,
    std::string cancoder_bus)
    : frc846::robot::GenericSubsystem<SwerveModuleReadings, SwerveModuleTarget>(
          parent, loc),
      drive_{motor_types, drive_params},
      steer_{motor_types, steer_params},
      cancoder_{cancoder_id, cancoder_bus} {
  drive_helper_.SetConversion(drive_reduction);
  steer_helper_.SetConversion(steer_reduction);

  drive_helper_.bind(&drive_);
  steer_helper_.bind(&steer_);
}

void SwerveModule::Setup() {
  drive_.Setup();
  steer_.Setup();

  // TODO: finish. Refer to howler_monkey.
}

SwerveModuleTarget SwerveModule::ZeroTarget() const {
  return SwerveModuleOLControlTarget{0.0, 0_deg};
}

bool SwerveModule::VerifyHardware() {
  bool ok = true;
  // TODO: Add a verify hardware to motor controllers.
  return ok;
}

SwerveModuleReadings SwerveModule::ReadFromHardware() {
  SwerveModuleReadings readings;
  readings.vel = drive_helper_.GetVelocity();
  readings.drive_pos = drive_helper_.GetPosition();
  readings.steer_pos = steer_helper_.GetPosition();
  return readings;

  // TODO: recheck
}

void SwerveModule::WriteToHardware(SwerveModuleTarget target) {
  // TODO: finish. Refer to howler_monkey for open-loop.
}

void SwerveModule::SetSteerGains(frc846::control::base::MotorGains gains) {
  steer_.SetGains(gains);
}

}  // namespace frc846::robot::swerve