#include "frc846/robot/swerve/drivetrain.h"

#include <thread>

#include "frc846/robot/swerve/control/swerve_ol_calculator.h"
#include "frc846/robot/swerve/swerve_module.h"

namespace frc846::robot::swerve {

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainConfigs configs)
    : GenericSubsystem{"SwerveDrivetrain"},
      configs_{configs},
      modules_{},
      navX_{configs.navX_connection_mode, studica::AHRS::k200Hz} {
  for (int i = 0; i < 4; i++) {
    modules_[i] = new SwerveModuleSubsystem{*this,
        configs_.module_unique_configs[i], configs_.module_common_config};
  }

  RegisterPreference("steer_gains/_kP", 4.0);
  RegisterPreference("steer_gains/_kI", 0.0);
  RegisterPreference("steer_gains/_kD", 0.0);
  RegisterPreference("steer_gains/_kF", 0.0);

  RegisterPreference("bearing_gains/_kP", 0.5);
  RegisterPreference("bearing_gains/_kI", 0.0);
  RegisterPreference("bearing_gains/_kD", 0.0);

  RegisterPreference("max_speed", 15_fps);
  RegisterPreference("max_omega", units::degrees_per_second_t{180});

  RegisterPreference("odom_fudge_factor", 0.875);

  RegisterPreference("steer_lag", 0.05_s);

  odometry_.setConstants({});
  ol_calculator_.setConstants({
      .wheelbase_horizontal_dim = configs.wheelbase_horizontal_dim,
      .wheelbase_forward_dim = configs.wheelbase_forward_dim,
  });
}

void DrivetrainSubsystem::Setup() {
  frc846::control::base::MotorGains steer_gains{
      GetPreferenceValue_double("steer_gains/_kP"),
      GetPreferenceValue_double("steer_gains/_kI"),
      GetPreferenceValue_double("steer_gains/_kD"),
      GetPreferenceValue_double("steer_gains/_kF")};
  for (SwerveModuleSubsystem* module : modules_) {
    module->InitByParent();
    module->Setup();
    module->SetSteerGains(steer_gains);
  }
  ZeroBearing();
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  return DrivetrainOLControlTarget{{0_fps, 0_fps}, 0_deg_per_s};
}

bool DrivetrainSubsystem::VerifyHardware() {
  bool ok = true;
  for (SwerveModuleSubsystem* module : modules_) {
    ok &= module->VerifyHardware();
  }
  FRC846_VERIFY(ok, ok, "At least one module failed verification");
  return ok;
}

void DrivetrainSubsystem::ZeroBearing() {
  if (!is_initialized()) return;

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("Gyro zero attempt {}/{}", attempts, kMaxAttempts);
    if (navX_.IsConnected() && !navX_.IsCalibrating()) {
      navX_.ZeroYaw();
      Log("Zeroed bearing");

      for (SwerveModuleSubsystem* module : modules_) {
        module->ZeroWithCANcoder();
      }
      return;
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
  Error("Unable to zero after {} attempts", kMaxAttempts);
}

void DrivetrainSubsystem::SetCANCoderOffsets() {
  for (SwerveModuleSubsystem* module : modules_) {
    module->SetCANCoderOffset();
  }
}

units::degrees_per_second_t DrivetrainSubsystem::ApplyBearingPID(
    units::degree_t target_bearing) {
  units::degree_t bearing = GetReadings().pose.bearing;
  units::degrees_per_second_t yaw_rate = GetReadings().yaw_rate;

  units::degree_t error =
      frc846::math::CoterminalDifference(bearing, target_bearing);

  Graph("bearing_pid/bearing", bearing);
  Graph("bearing_pid/target", target_bearing);
  Graph("bearing_pid/error", error);
  Graph("bearing_pid/yaw_rate", yaw_rate);

  frc846::control::base::MotorGains gains{
      GetPreferenceValue_double("bearing_gains/_kP"),
      GetPreferenceValue_double("bearing_gains/_kI"),
      GetPreferenceValue_double("bearing_gains/_kD"), 0.0};

  return 1_deg_per_s *
         gains.calculate(error.to<double>(), 0.0, yaw_rate.to<double>(), 0.0);
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  units::degree_t bearing = navX_.GetAngle() * 1_deg;
  units::degrees_per_second_t yaw_rate = navX_.GetRate() * 1_deg_per_s;

  Graph("readings/bearing", bearing);
  Graph("readings/yaw_rate", yaw_rate);

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
      .position = odometry_
                      .calculate({bearing, steer_positions, drive_positions,
                          GetPreferenceValue_double("odom_fudge_factor")})
                      .position,
      .bearing = bearing,
      .velocity = velocity,
  };

  // TODO: consider bearing simulation

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);

  return {new_pose, yaw_rate};
}

frc846::math::VectorND<units::feet_per_second, 2>
DrivetrainSubsystem::compensateForSteerLag(
    frc846::math::VectorND<units::feet_per_second, 2> uncompensated) {
  units::degree_t steer_lag_compensation =
      -GetPreferenceValue_unit_type<units::second_t>("steer_lag") *
      GetReadings().yaw_rate;

  Graph("target/steer_lag_compensation", steer_lag_compensation);

  return uncompensated.rotate(steer_lag_compensation, true);
}

void DrivetrainSubsystem::WriteVelocitiesHelper(
    frc846::math::VectorND<units::feet_per_second, 2> velocity,
    units::degrees_per_second_t angular_velocity, bool cut_excess_steering,
    units::feet_per_second_t speed_limit) {
  units::degree_t bearing = GetReadings().pose.bearing;
  auto velocity_compensated = compensateForSteerLag(velocity);

  auto ol_calc_outputs = ol_calculator_.calculate({velocity_compensated,
      angular_velocity, bearing, speed_limit, cut_excess_steering});

  for (int i = 0; i < 4; i++) {
    modules_[i]->SetTarget(SwerveModuleOLControlTarget{
        ol_calc_outputs.drive_outputs[i], ol_calc_outputs.steer_outputs[i]});
  }
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  for (int i = 0; i < 4; i++) {
    modules_[i]->SetSteerGains({GetPreferenceValue_double("steer_gains/_kP"),
        GetPreferenceValue_double("steer_gains/_kI"),
        GetPreferenceValue_double("steer_gains/_kD"),
        GetPreferenceValue_double("steer_gains/_kF")});
  }

  if (DrivetrainOLControlTarget* ol_target =
          std::get_if<DrivetrainOLControlTarget>(&target)) {
    Graph("target/ol/velocity_x", ol_target->velocity[0]);
    Graph("target/ol/velocity_y", ol_target->velocity[1]);
    Graph("target/ol/angular_velocity", ol_target->angular_velocity);

    WriteVelocitiesHelper(ol_target->velocity, ol_target->angular_velocity,
        false,
        GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed"));
  } else if (DrivetrainAccelerationControlTarget* accel_target =
                 std::get_if<DrivetrainAccelerationControlTarget>(&target)) {
    Graph(
        "target/accel/linear_acceleration", accel_target->linear_acceleration);
    Graph("target/accel/accel_dir", accel_target->accel_dir);
    Graph("target/angular_velocity", accel_target->angular_velocity);

    auto motor_specs = frc846::control::base::MotorSpecificationPresets::get(
        configs_.module_common_config.motor_types);

    units::feet_per_second_t true_max_speed =
        motor_specs.free_speed * configs_.module_common_config.drive_reduction;
    units::feet_per_second_t accel_buffer =
        accel_target->linear_acceleration / configs_.max_accel * true_max_speed;

    auto vel_new_target = GetReadings().pose.velocity +
                          frc846::math::VectorND<units::feet_per_second, 2>{
                              accel_buffer, accel_target->accel_dir, true};

    units::feet_per_second_t speed_limit =
        GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed");
    if (accel_target->speed_limit >= 1_fps)
      speed_limit = accel_target->speed_limit;
    WriteVelocitiesHelper(
        vel_new_target, accel_target->angular_velocity, true, speed_limit);
  }

  for (int i = 0; i < 4; i++)
    modules_[i]->UpdateHardware();
}

}  // namespace frc846::robot::swerve