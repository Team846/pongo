#include "frc846/robot/swerve/drivetrain.h"

#include "frc846/robot/swerve/swerve_module.h"

namespace frc846::robot::swerve {

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainConfigs configs)
    : GenericSubsystem{"SwerveDrivetrain"}, configs_{configs}, modules_{} {
  // if (configs_.navX_connection_mode == NavX_connection_type::kSPI) {
  //   navX_ = new AHRS{frc::SPI::Port::kMXP};
  // } else if (configs_.navX_connection_mode == NavX_connection_type::kSerial)
  // {
  //   navX_ = new AHRS{frc::SerialPort::Port::kMXP};
  // } else {
  //   throw std::runtime_error("Invalid navX connection mode");
  // }

  // TODO: finish

  std::array<std::string, 4> locations{"FR", "FL", "BL", "BR"};
  std::array<int, 4> can_ids {}

  for (int i = 0; i < 4; i++) {
    modules_[i] =
        new SwerveModuleSubsystem{*this, "Module" + std::to_string(i)};
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
  // TODO: finish
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
  units::degree_t bearing = navX_->GetAngle() * 1_deg;

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

  frc846::robot::swerve::odometry::SwervePose new_pose{
      .position =
          odometry_.calculate({bearing, steer_positions, drive_positions})
              .position,
      .bearing = bearing,
      .velocity = velocity};

  return {new_pose};
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  // TODO: finish
  if (DrivetrainOLControlTarget* ol_target =
          std::get_if<DrivetrainOLControlTarget>(&target)) {
    units::degree_t bearing = navX_->GetAngle() * 1_deg;

    std::array<units::feet_per_second_t, 4> module_speeds;
    std::array<units::degree_t, 4> module_angles;
    for (int i = 0; i < 4; i++) {
      frc846::math::VectorND<units::feet_per_second, 2> module_vel =
          ol_target->velocity;

      module_vel = module_vel.rotate(-bearing);

      module_speeds[i] = module_vel.magnitude();
      module_angles[i] = module_vel.angle();
    }

    for (int i = 0; i < 4; i++) {
      SwerveModuleOLControlTarget module_target{
          module_speeds[i].to<double>(), module_angles[i]};

      modules_[i]->UpdateHardware(module_target);
    }
  } else if (DrivetrainAccelerationControlTarget* accel_target =
                 std::get_if<DrivetrainAccelerationControlTarget>(&target)) {
    DrivetrainReadings readings = GetReadings();
    units::degree_t bearing = readings.pose.bearing;

    frc846::math::VectorND<units::feet_per_second, 2> target_velocity =
        readings.pose.velocity +
        frc846::math::VectorND<units::feet_per_second, 2>{
            accel_target->linear_acceleration};

    DrivetrainOLControlTarget ol_target{target_velocity};
    WriteToHardware(ol_target);
  }
}

}  // namespace frc846::robot::swerve