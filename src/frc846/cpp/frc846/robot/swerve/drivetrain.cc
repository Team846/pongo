#include "frc846/robot/swerve/drivetrain.h"

#include "frc846/robot/swerve/swerve_module.h"

namespace frc846::robot::swerve {

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainConfigs configs)
    : GenericSubsystem{"SwerveDrivetrain"},
      configs_{configs},
      modules_{},
      navX_{frc::SerialPort::kMXP} {
  for (int i = 0; i < 4; i++) {
    modules_[i] = new SwerveModuleSubsystem{*this,
        configs_.module_unique_configs[i], configs_.module_common_config};
  }

  RegisterPreference("steer_gains/kP", 0.3);
  RegisterPreference("steer_gains/kI", 0.0);
  RegisterPreference("steer_gains/kD", 0.0);
  RegisterPreference("steer_gains/kF", 0.0);
}

void DrivetrainSubsystem::Setup() {
  frc846::control::base::MotorGains steer_gains{
      GetPreferenceValue_double("steer_gains/kP"),
      GetPreferenceValue_double("steer_gains/kI"),
      GetPreferenceValue_double("steer_gains/kD"),
      GetPreferenceValue_double("steer_gains/kF")};
  for (SwerveModuleSubsystem* module : modules_) {
    module->InitByParent();
    module->Setup();
    module->SetSteerGains(steer_gains);
    module->ZeroWithCANcoder();
  }
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  return DrivetrainOLControlTarget{{0_fps, 0_fps}};
}

bool DrivetrainSubsystem::VerifyHardware() {
  bool ok = true;
  for (SwerveModuleSubsystem* module : modules_) {
    ok &= module->VerifyHardware();
  }
  FRC846_VERIFY(ok, ok, "At least one module failed verification");
  return ok;
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  units::degree_t bearing = navX_.GetAngle() * 1_deg;

  Graph("readings/bearing", bearing);

  frc846::math::VectorND<units::inch, 4> drive_positions{
      0_in, 0_in, 0_in, 0_in};
  std::array<units::degree_t, 4> steer_positions{};

  frc846::math::VectorND<units::feet_per_second, 2> velocity{0_fps, 0_fps};

  for (int i = 0; i < 4; i++) {
    modules_[i]->UpdateReadings();
    SwerveModuleReadings r = modules_[i]->GetReadings();

    drive_positions[i] = r.drive_pos;
    steer_positions[i] = r.steer_pos;

    velocity += (frc846::math::VectorND<units::feet_per_second, 2>{
        r.vel, r.steer_pos + bearing, true});
  }

  velocity /= 4.0;

  Graph("readings/velocity_x", velocity[0]);
  Graph("readings/velocity_y", velocity[1]);

  frc846::robot::swerve::odometry::SwervePose new_pose{
      .position =
          odometry_.calculate({bearing, steer_positions, drive_positions})
              .position,
      .bearing = bearing,
      .velocity = velocity};

  // TODO: consider bearing simulation

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);

  return {new_pose};
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  // TODO: finish
  if (DrivetrainOLControlTarget* ol_target =
          std::get_if<DrivetrainOLControlTarget>(&target)) {
    units::degree_t bearing = navX_.GetAngle() * 1_deg;

    /*
    TODO: For each module, find the target direction and speed based on velocity
    and turn speed. Then, construct a SwerveModuleOLControlTarget and write it
    to the module.
    */

    for (int i = 0; i < 4; i++) {
      // modules_[i]->SetTarget(module_target);
      // modules_[i]->UpdateHardware();
    }
  } else if (DrivetrainAccelerationControlTarget* accel_target =
                 std::get_if<DrivetrainAccelerationControlTarget>(&target)) {
    throw std::runtime_error("Acceleration control not yet implemented");
    // TODO: finish later
  }
}

}  // namespace frc846::robot::swerve