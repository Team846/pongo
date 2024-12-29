#include "frc846/robot/swerve/drivetrain.h"

namespace frc846::robot::swerve {

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainConfigs configs, bool enable)
    : GenericSubsystem{"SwerveDrivetrain"}, configs_{configs}, modules_{} {
  // TODO: finish
}

void DrivetrainSubsystem::Setup() {
  // TODO: finish
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  return DrivetrainOLControlTarget{{0_fps, 0_fps}};
}

bool DrivetrainSubsystem::VerifyHardware() {
  bool ok = true;
  // TODO: finish
  return ok;
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  DrivetrainReadings readings;
  // TODO: finish
  return readings;
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  // TODO: finish
}

}  // namespace frc846::robot::swerve