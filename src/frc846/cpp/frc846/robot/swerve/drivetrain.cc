#include "frc846/robot/swerve/drivetrain.h"

#include <thread>

#include "frc846/math/constants.h"
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
  RegisterPreference("bearing_gains/deadband", 3.0_deg_per_s);

  RegisterPreference("lock_gains/_kP", 0.5);
  RegisterPreference("lock_gains/_kI", 0.0);
  RegisterPreference("lock_gains/_kD", 0.0);
  RegisterPreference("lock_gains/deadband", 2_in);
  RegisterPreference("lock_adj_rate", 0.05_in);
  RegisterPreference("lock_max_speed", 7_fps);

  RegisterPreference("drive_to_subtract", 2_in);

  RegisterPreference("max_speed", 15_fps);
  RegisterPreference("max_omega", units::degrees_per_second_t{180});
  RegisterPreference("max_omega_cut", units::degrees_per_second_t{40});

  RegisterPreference("odom_fudge_factor", 0.875);
  RegisterPreference("odom_variance", 0.2);

  RegisterPreference("steer_lag", 0.05_s);

  RegisterPreference("pose_estimator/pose_variance", 0.01);
  RegisterPreference("pose_estimator/velocity_variance", 1.0);
  RegisterPreference("pose_estimator/accel_variance", 1.0);

  RegisterPreference("april_tags/april_variance_coeff", 0.33);
  RegisterPreference("april_tags/fudge_latency", 20_ms);

  RegisterPreference("rc_control_speed", 2.5_fps);

  RegisterPreference("accel_spike_thresh", 45_fps_sq);
  RegisterPreference("max_past_accel_spike", 25);
  RegisterPreference("accel_vel_stopped_thresh", 0.7_fps);
  RegisterPreference("vel_stopped_thresh", 1.0_fps);
  RegisterPreference("stopped_num_loops", 25);

  RegisterPreference("reef_drive_early", 12_in);
  RegisterPreference("reef_drive_fvel", 1_fps);
        
  odometry_.setConstants({});
  ol_calculator_.setConstants({
      .wheelbase_horizontal_dim = configs.wheelbase_horizontal_dim,
      .wheelbase_forward_dim = configs.wheelbase_forward_dim,
  });

  std::vector<std::shared_ptr<nt::NetworkTable>> april_tables = {};
  for (int i = 0; i < configs.cams; i++) {
    april_tables.push_back(nt::NetworkTableInstance::GetDefault().GetTable(
        "AprilTagsCam" + std::to_string(i + 1)));
  }
  tag_pos_calculator.setConstants({.tag_locations = configs.april_locations,
      .camera_x_offsets = configs.camera_x_offsets,
      .camera_y_offsets = configs.camera_y_offsets,
      .cams = configs.cams,
      .april_tables = april_tables});
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

  pose_estimator.SetPoint(
      {GetReadings().april_point[0], GetReadings().april_point[1]});
}

void DrivetrainSubsystem::SetPosition(frc846::math::Vector2D position) {
  odometry_.SetPosition(position);
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
      frc846::math::CoterminalDifference(target_bearing, bearing);

  Graph("bearing_pid/bearing", bearing);
  Graph("bearing_pid/target", target_bearing);
  Graph("bearing_pid/error", error);
  Graph("bearing_pid/yaw_rate", yaw_rate);

  frc846::control::base::MotorGains gains{
      GetPreferenceValue_double("bearing_gains/_kP"),
      GetPreferenceValue_double("bearing_gains/_kI"),
      GetPreferenceValue_double("bearing_gains/_kD"), 0.0};

  double raw_output =
      gains.calculate(error.to<double>(), 0.0, yaw_rate.to<double>(), 0.0);

  Graph("bearing_pid/raw_output (c.dps)", raw_output);

  units::degrees_per_second_t output =
      1_deg_per_s *
      frc846::math::HorizontalDeadband(raw_output,
          GetPreferenceValue_unit_type<units::degrees_per_second_t>(
              "bearing_gains/deadband")
              .to<double>(),
          GetPreferenceValue_unit_type<units::degrees_per_second_t>("max_omega")
              .to<double>());

  Graph("bearing_pid/output", output);

  return output;
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  pose_estimator.Update(
      GetPreferenceValue_double("pose_estimator/pose_variance"),
      GetPreferenceValue_double("pose_estimator/velocity_variance"),
      GetPreferenceValue_double("pose_estimator/accel_variance"));

  units::degree_t bearing = navX_.GetAngle() * 1_deg;
  units::degrees_per_second_t yaw_rate = navX_.GetRate() * 1_deg_per_s;

  frc846::math::VectorND<units::feet_per_second_squared, 2> accl{
      navX_.GetWorldLinearAccelX() * frc846::math::constants::physics::g,
      navX_.GetWorldLinearAccelY() * frc846::math::constants::physics::g};
  Graph("navX/acclX", accl[0]);
  Graph("navX/acclY", accl[1]);
  pose_estimator.AddAccelerationMeasurement(accl);

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

  frc846::math::Vector2D delta_pos =
      new_pose.position - GetReadings().pose.position;
  pose_estimator.AddOdometryMeasurement(
      {delta_pos[0], delta_pos[1]}, GetPreferenceValue_double("odom_variance"));

  frc846::robot::swerve::odometry::SwervePose estimated_pose{
      .position = {pose_estimator.position()[0], pose_estimator.position()[1]},
      .bearing = bearing,
      .velocity = pose_estimator.velocity(),
  };  // Initialize estimated pose

  frc846::robot::calculators::ATCalculatorOutput tag_pos =
      tag_pos_calculator.calculate({estimated_pose, yaw_rate,
          GetPreferenceValue_double("april_tags/april_variance_coeff"),
          GetPreferenceValue_unit_type<units::millisecond_t>(
              "april_tags/fudge_latency")});

  if (tag_pos.variance >= 0) {
    pose_estimator.AddVisionMeasurement(
        {tag_pos.pos[0], tag_pos.pos[1]}, tag_pos.variance);
  }

  Graph("april_tags/april_pos_x", tag_pos.pos[0]);
  Graph("april_tags/april_pos_y", tag_pos.pos[1]);
  Graph("april_tags/april_variance", tag_pos.variance);

  if (first_loop) {
    pose_estimator.SetPoint({tag_pos.pos[0], tag_pos.pos[1]});
    first_loop = false;
  }

  estimated_pose = {
      .position = {pose_estimator.position()[0], pose_estimator.position()[1]},
      .bearing = bearing,
      .velocity = pose_estimator.velocity(),
  };  // Update estimated pose again with vision data

  Graph("estimated_pose/position_x", estimated_pose.position[0]);
  Graph("estimated_pose/position_y", estimated_pose.position[1]);
  Graph("estimated_pose/velocity_x", estimated_pose.velocity[0]);
  Graph("estimated_pose/velocity_y", estimated_pose.velocity[1]);
  Graph("estimated_pose/variance", pose_estimator.getVariance());

  // TODO: consider bearing simulation

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);

  units::meters_per_second_squared_t accel_x{navX_.GetWorldLinearAccelX()};
  units::meters_per_second_squared_t accel_y{navX_.GetWorldLinearAccelY()};
  units::meters_per_second_squared_t accel_z{navX_.GetWorldLinearAccelZ()};
  Graph("readings/accel_x", accel_x);
  Graph("readings/accel_y", accel_y);
  Graph("readings/accel_z", accel_z);

  units::meters_per_second_squared_t accel_mag = units::math::sqrt(
      accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
  Graph("readings/accel_mag", accel_mag);

  last_accel_spike_ += 1;
  if (accel_mag >=
      GetPreferenceValue_unit_type<units::feet_per_second_squared_t>(
          "accel_spike_thresh")) {
    last_accel_spike_ = 0;
  }
  Graph("readings/last_accel_spike", last_accel_spike_);

  units::meters_per_second_t accel_vel_x{navX_.GetVelocityX()};
  units::meters_per_second_t accel_vel_y{navX_.GetVelocityY()};
  units::meters_per_second_t accel_vel_z{navX_.GetVelocityZ()};

  Graph("readings/accel_vel_x", accel_vel_x);
  Graph("readings/accel_vel_y", accel_vel_y);
  Graph("readings/accel_vel_z", accel_vel_z);

  units::meters_per_second_t accel_vel = units::math::sqrt(
      units::math::pow<2>(accel_vel_x) + units::math::pow<2>(accel_vel_y) +
      units::math::pow<2>(accel_vel_z));

  Graph("readings/accel_vel", accel_vel);

  return {new_pose, tag_pos.pos, estimated_pose, yaw_rate, accel_mag, accel_vel, last_accel_spike_};
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
    Graph("target/true_max_speed", true_max_speed);
    units::feet_per_second_t accel_buffer =
        accel_target->linear_acceleration / configs_.max_accel * true_max_speed;
    Graph("target/accel_buffer", accel_buffer);

    auto vel_new_target = GetReadings().pose.velocity +
                          frc846::math::VectorND<units::feet_per_second, 2>{
                              accel_buffer, accel_target->accel_dir, true};

    units::degrees_per_second_t angular_vel = units::math::min(
        units::math::max(accel_target->angular_velocity,
            -GetPreferenceValue_unit_type<units::degrees_per_second_t>(
                "max_omega_cut")),
        GetPreferenceValue_unit_type<units::degrees_per_second_t>(
            "max_omega_cut"));

    units::feet_per_second_t speed_limit =
        GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed");
    if (accel_target->speed_limit >= 1_fps)
      speed_limit = accel_target->speed_limit;
    WriteVelocitiesHelper(vel_new_target, angular_vel, true, speed_limit);
  }

  for (int i = 0; i < 4; i++)
    modules_[i]->UpdateHardware();
}

}  // namespace frc846::robot::swerve