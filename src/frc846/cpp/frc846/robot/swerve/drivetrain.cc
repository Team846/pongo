#include "frc846/robot/swerve/drivetrain.h"

#include <thread>

#include "frc/RobotBase.h"
#include "frc846/math/constants.h"
#include "frc846/math/fieldpoints.h"
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

  RegisterPreference("steer_gains/_kP", 2.0);
  RegisterPreference("steer_gains/_kI", 0.0);
  RegisterPreference("steer_gains/_kD", 0.0);
  RegisterPreference("steer_gains/_kF", 0.0);

  RegisterPreference("bearing_gains/_kP", 9);
  RegisterPreference("bearing_gains/_kI", 0.0);
  RegisterPreference("bearing_gains/_kD", -0.6);
  RegisterPreference("bearing_gains/deadband", 3.0_deg_per_s);

  RegisterPreference("lock_gains/_kP", -0.7);
  RegisterPreference("lock_gains/_kI", 0.0);
  RegisterPreference("lock_gains/_kD", 0.036);
  RegisterPreference("lock_gains/deadband", 0.5_in);
  RegisterPreference("lock_adj_rate", 0.05_in);
  RegisterPreference("lock_max_speed", 9_fps);
  RegisterPreference("auto_max_speed", 15_fps);

  RegisterPreference("drive_to_subtract", 2_in);

  RegisterPreference("april_bearing_latency", 0_ms);
  RegisterPreference("drive_latency", 0_ms);

  RegisterPreference("max_speed", 15_fps);
  RegisterPreference("max_omega", units::degrees_per_second_t{180});
  RegisterPreference("max_omega_cut", units::degrees_per_second_t{40});

  RegisterPreference("odom_fudge_factor", 1.077);
  RegisterPreference("odom_variance", 0.2);

  RegisterPreference("steer_lag", 0.05_s);
  RegisterPreference("bearing_latency", 0.01_s);

  RegisterPreference("pose_estimator/pose_variance", 0.1);
  RegisterPreference("pose_estimator/velocity_variance", 1.0);
  RegisterPreference("pose_estimator/accel_variance", 1.0);
  RegisterPreference("pose_estimator/override", false);

  RegisterPreference("april_tags/april_variance_coeff", 0.08);
  RegisterPreference("april_tags/triangular_variance_coeff", 0.000139);
  RegisterPreference("april_tags/fudge_latency1", 155.0_ms);
  RegisterPreference("april_tags/fudge_latency2", 70.0_ms);

  RegisterPreference("rc_control_speed", 2.5_fps);

  RegisterPreference("accel_spike_thresh", 45_fps_sq);
  RegisterPreference("max_past_accel_spike", 25);
  RegisterPreference("accel_vel_stopped_thresh", 0.7_fps);
  RegisterPreference("vel_stopped_thresh", 0.7_fps);
  RegisterPreference("stopped_num_loops", 25);

  RegisterPreference("drive_to_point/kC", 5.0);
  RegisterPreference("drive_to_point/kA", 0.05);
  RegisterPreference("drive_to_point/kE", 5.0);
  RegisterPreference("drive_to_point/threshold", 6_in);

  RegisterPreference("override_at_auto", true);

  RegisterPreference("source_max_speed", 3_fps);
  RegisterPreference("source_coast_threshold", 10_in);
  RegisterPreference("source_coast_speed", 2_fps);
  RegisterPreference("use_source_assist", true);

  RegisterPreference("net_auto_align/prepoint", 65_in);
  RegisterPreference("net_auto_align/scorepoint", 35_in);

  odometry_.setConstants(
      {.forward_wheelbase_dim = configs.wheelbase_forward_dim,
          .horizontal_wheelbase_dim = configs.wheelbase_horizontal_dim});
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

  if (!frc::DriverStation::IsAutonomous()) {
    if (frc::DriverStation::GetAlliance() ==
        frc::DriverStation::Alliance::kBlue)
      bearing_offset_ = 180_deg;
    else
      bearing_offset_ = 0_deg;
  }
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("Gyro zero attempt {}/{}", attempts, kMaxAttempts);
    if (navX_.IsConnected() && !navX_.IsCalibrating()) {
      navX_.ZeroYaw();
      Log("Zeroed bearing");

      // for (SwerveModuleSubsystem* module : modules_) {
      //   module->ZeroWithCANcoder();
      // }
      return;
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
  Error("Unable to zero after {} attempts, forcing zero", kMaxAttempts);

  navX_.ZeroYaw();
  // for (SwerveModuleSubsystem* module : modules_) {
  //   module->ZeroWithCANcoder();
  // }

  pose_estimator.SetPoint(
      {GetReadings().april_point[0], GetReadings().april_point[1]});
}

void DrivetrainSubsystem::SetBearing(units::degree_t bearing) {
  bearing_offset_ = bearing - (GetReadings().pose.bearing - bearing_offset_);
  // Log("setting bearing to {}", bearing);
}

void DrivetrainSubsystem::SetPosition(frc846::math::Vector2D position) {
  odometry_.SetPosition(position);
  // Log("setting position x to {} and position y to {}", position[0],
  //     position[1]);
  pose_estimator.SetPoint({position[0], position[1]});
}

void DrivetrainSubsystem::SetOdomBearing(units::degree_t odom_bearing) {
  odometry_.SetOdomBearing(odom_bearing);
  // Log("setting odom bearing to {}", odom_bearing);
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

  // Graph("bearing_pid/bearing", bearing);
  // Graph("bearing_pid/target", target_bearing);
  Graph("bearing_pid/error", error);
  // Graph("bearing_pid/yaw_rate", yaw_rate);

  frc846::control::base::MotorGains gains{
      GetPreferenceValue_double("bearing_gains/_kP"),
      GetPreferenceValue_double("bearing_gains/_kI"),
      GetPreferenceValue_double("bearing_gains/_kD"), 0.0};

  double raw_output =
      gains.calculate(error.to<double>(), 0.0, yaw_rate.to<double>(), 0.0);

  // Graph("bearing_pid/raw_output (c.dps)", raw_output);

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
  bearing += bearing_offset_;

  units::degrees_per_second_t yaw_rate = navX_.GetRate() * 1_deg_per_s;

  bearing += GetPreferenceValue_unit_type<units::second_t>("bearing_latency") *
             yaw_rate;

  if (frc::RobotBase::IsSimulation()) { bearing = odometry_.GetOdomBearing(); }

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

  Graph("readings/velocity_mag", velocity.magnitude());

  units::feet_per_second_squared_t odom_accel_x =
      1_fps_sq * accel_x_diff.Calculate(velocity[0].to<double>());
  units::feet_per_second_squared_t odom_accel_y =
      1_fps_sq * accel_y_diff.Calculate(velocity[1].to<double>());
  odom_accel_x = 1_fps_sq * accel_x_smooth.Calculate(odom_accel_x.to<double>());
  odom_accel_y = 1_fps_sq * accel_y_smooth.Calculate(odom_accel_y.to<double>());

  Graph("readings/odom_accel_x", odom_accel_x);
  Graph("readings/odom_accel_y", odom_accel_y);
  Graph("readings/odom_accel_mag",
      units::math::sqrt(
          odom_accel_x * odom_accel_x + odom_accel_y * odom_accel_y));

  frc846::robot::swerve::odometry::SwerveOdometryOutput odom_output =
      odometry_.calculate({bearing, steer_positions, drive_positions,
          GetPreferenceValue_double("odom_fudge_factor")});

  frc846::robot::swerve::odometry::SwervePose new_pose{
      .position = odom_output.position,
      .bearing = bearing,
      .velocity = velocity,
  };

  frc846::math::Vector2D delta_pos =
      new_pose.position - GetReadings().pose.position;

  if (delta_pos.magnitude() < 10_in) {
    pose_estimator.AddOdometryMeasurement({delta_pos[0], delta_pos[1]},
        GetPreferenceValue_double("odom_variance"));
  }

  frc846::robot::swerve::odometry::SwervePose estimated_pose{
      .position = {pose_estimator.position()[0], pose_estimator.position()[1]},
      .bearing = bearing,
      .velocity = pose_estimator.velocity(),
  };  // Initialize estimated pose

  frc846::robot::calculators::ATCalculatorOutput tag_pos =
      tag_pos_calculator.calculate({new_pose, GetReadings().pose, yaw_rate,
          GetPreferenceValue_double("april_tags/april_variance_coeff"),
          GetPreferenceValue_double("april_tags/triangular_variance_coeff"),
          {GetPreferenceValue_unit_type<units::millisecond_t>(
               "april_tags/fudge_latency1"),
              GetPreferenceValue_unit_type<units::millisecond_t>(
                  "april_tags/fudge_latency1")},
          GetPreferenceValue_unit_type<units::millisecond_t>(
              "april_bearing_latency")});

  if (tag_pos.variance >= 0) {
    pose_estimator.AddVisionMeasurement(
        {tag_pos.pos[0], tag_pos.pos[1]}, tag_pos.variance);
    see_tag_counter_ = 0;
  } else {
    see_tag_counter_++;
  }
  Graph("april_tags/see_tag_counter", see_tag_counter_);

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

  if (frc::RobotBase::IsSimulation()) {
    estimated_pose.position = odom_output.position;
    estimated_pose.velocity = velocity;
  }

  if ((frc::DriverStation::IsAutonomous() ||
          frc::DriverStation::IsAutonomousEnabled()) &&
      GetPreferenceValue_bool("override_at_auto")) {
    estimated_pose = new_pose;
    Graph("overriding_kalman_pose_auton", true);
  } else {
    Graph("overriding_kalman_pose_auton", false);
  }

  if (GetPreferenceValue_bool("pose_estimator/override")) {
    estimated_pose = new_pose;
  }

  Graph("estimated_pose/position_x", estimated_pose.position[0]);
  Graph("estimated_pose/position_y", estimated_pose.position[1]);
  Graph("estimated_pose/velocity_x", estimated_pose.velocity[0]);
  Graph("estimated_pose/velocity_y", estimated_pose.velocity[1]);
  Graph("estimated_pose/variance", pose_estimator.getVariance());

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);
  Graph("readings/odom_bearing", odom_output.odom_bearing);

  frc846::math::VectorND<units::feet_per_second_squared, 2> accl{
      navX_.GetWorldLinearAccelX() * frc846::math::constants::physics::g,
      navX_.GetWorldLinearAccelY() * frc846::math::constants::physics::g};

  Graph("readings/accel_x", accl[0]);
  Graph("readings/accel_y", accl[1]);

  accl.rotate(bearing_offset_);
  pose_estimator.AddAccelerationMeasurement(accl);

  units::meters_per_second_squared_t accel_mag =
      units::math::sqrt(accl[0] * accl[0] + accl[1] * accl[1]);
  // Graph("readings/accel_mag", accel_mag);

  last_accel_spike_ += 1;
  if (accel_mag >=
      GetPreferenceValue_unit_type<units::feet_per_second_squared_t>(
          "accel_spike_thresh")) {
    last_accel_spike_ = 0;
  }
  // Graph("readings/last_accel_spike", last_accel_spike_);

  units::meters_per_second_t accel_vel_x{navX_.GetVelocityX()};
  units::meters_per_second_t accel_vel_y{navX_.GetVelocityY()};
  units::meters_per_second_t accel_vel_z{navX_.GetVelocityZ()};

  // Graph("readings/accel_vel_x", accel_vel_x);
  // Graph("readings/accel_vel_y", accel_vel_y);
  // Graph("readings/accel_vel_z", accel_vel_z);

  units::meters_per_second_t accel_vel = units::math::sqrt(
      units::math::pow<2>(accel_vel_x) + units::math::pow<2>(accel_vel_y) +
      units::math::pow<2>(accel_vel_z));

  // Graph("readings/accel_vel", accel_vel);

  // Record the current pose if path recording is active
  if (path_logger_.IsRecording()) { path_logger_.RecordPose(estimated_pose); }

  return {new_pose, tag_pos.pos, estimated_pose, yaw_rate, accel_mag, accel_vel,
      last_accel_spike_, see_tag_counter_};
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

frc846::math::VectorND<units::feet_per_second, 2>
DrivetrainSubsystem::accelClampHelper(
    frc846::math::VectorND<units::feet_per_second, 2> velocity,
    units::feet_per_second_squared_t accel_clamp) {
  if (accel_clamp < 5_fps_sq) return velocity;

  auto motor_specs = frc846::control::base::MotorSpecificationPresets::get(
      configs_.module_common_config.motor_types);

  frc846::wpilib::unit_ohm winding_res = 12_V / motor_specs.stall_current;

  double max_accel_corr_factor =
      (winding_res /
          (configs_.module_common_config.avg_resistance + winding_res))
          .to<double>();

  units::feet_per_second_t accel_buffer =
      accel_clamp / (max_accel_corr_factor * configs_.max_accel) *
      (motor_specs.free_speed * configs_.module_common_config.drive_reduction);

  // TODO: batt voltage compensation

  auto delta =
      velocity.magnitude() - GetReadings().estimated_pose.velocity.magnitude();
  if (units::math::abs(delta) > accel_buffer) {
    velocity = GetReadings().estimated_pose.velocity +
               frc846::math::VectorND<units::feet_per_second, 2>{
                   units::math::copysign(accel_buffer, delta),
                   velocity.angle(true), true};
  }
  return velocity;
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

  // Graph("target/ol/velocity_x", target.velocity[0]);
  // Graph("target/ol/velocity_y", target.velocity[1]);
  // Graph("target/ol/angular_velocity", target.angular_velocity);

  units::degrees_per_second_t cut_angular_vel = units::math::min(
      units::math::max(target.angular_velocity,
          -GetPreferenceValue_unit_type<units::degrees_per_second_t>(
              "max_omega_cut")),
      GetPreferenceValue_unit_type<units::degrees_per_second_t>(
          "max_omega_cut"));

  WriteVelocitiesHelper(accelClampHelper(target.velocity, target.accel_clamp),
      target.cut_excess_steering ? cut_angular_vel : target.angular_velocity,
      target.cut_excess_steering,
      GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed"));

  for (int i = 0; i < 4; i++)
    modules_[i]->UpdateHardware();
}

void DrivetrainSubsystem::StartPathRecording(const std::string& filename) {
  path_logger_.StartRecording(filename);
}

bool DrivetrainSubsystem::StopPathRecording() {
  return path_logger_.StopRecording();
}

bool DrivetrainSubsystem::IsPathRecording() const {
  return path_logger_.IsRecording();
}

}  // namespace frc846::robot::swerve