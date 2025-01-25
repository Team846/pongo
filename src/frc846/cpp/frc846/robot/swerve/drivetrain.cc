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

  RegisterPreference("align_gains/_kP", 0.6);
  RegisterPreference("align_gains/_kD", 0.1);

  RegisterPreference("max_speed", 15_fps);
  RegisterPreference("max_omega", units::degrees_per_second_t{180});

  RegisterPreference("odom_fudge_factor", 0.875);
  RegisterPreference("steer_lag", 0.05_s);

  RegisterPreference("pose_estimator/pose_variance", 0.01);
  RegisterPreference("pose_estimator/velocity_variance", 1000.0);
  RegisterPreference("pose_estimator/acceleration_variance", 1);

  RegisterPreference("april_tags/april_variance_coeff", 0.33);
  RegisterPreference("april_tags/fudge_latency", 20_ms);

  odometry_.setConstants({});
  ol_calculator_.setConstants({
      .wheelbase_horizontal_dim = configs.wheelbase_horizontal_dim,
      .wheelbase_forward_dim = configs.wheelbase_forward_dim,
  });

  std::vector<std::shared_ptr<nt::NetworkTable>> april_tables = {};
  for (int i = 0; i < configs.cams; i++) {
    april_tables.push_back(
        nt::NetworkTableInstance::GetDefault().GetTable("AprilTagsCam" + std::to_string(i+1)));
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

void DrivetrainSubsystem::SetCANCoderOffsets() {
  for (SwerveModuleSubsystem* module : modules_) {
    module->SetCANCoderOffset();
  }
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  pose_estimator.Update(
      GetPreferenceValue_double("pose_estimator/pose_variance"),
      GetPreferenceValue_double("pose_estimator/velocity_variance"),
      GetPreferenceValue_double("pose_estimator/acceleration_variance"));

  units::degree_t bearing = navX_.GetAngle() * 1_deg;
  units::degrees_per_second_t yaw_rate = navX_.GetRate() * 1_deg_per_s;

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
      .position = odometry_
          .calculate({bearing, steer_positions, drive_positions,
              GetPreferenceValue_double("odom_fudge_factor")})
          .position,
      .bearing = bearing,
      .velocity = velocity,
  };

  frc846::math::Vector2D delta_pos =
      new_pose.position - GetReadings().pose.position;
  pose_estimator.AddOdometryMeasurement({delta_pos[0], delta_pos[1]});

  frc846::robot::calculators::ATCalculatorOutput tag_pos =
      tag_pos_calculator.calculate({new_pose, yaw_rate,
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

  frc846::robot::swerve::odometry::SwervePose estimated_pose{
      .position = {pose_estimator.position()[0], pose_estimator.position()[1]},
      .bearing = bearing,
      .velocity = velocity,
  };

  Graph("estimated_pose/position_x", estimated_pose.position[0]);
  Graph("estimated_pose/position_y", estimated_pose.position[1]);
  Graph("estimated_pose/variance", pose_estimator.getVariance());

  // TODO: consider bearing simulation

  Graph("readings/position_x", new_pose.position[0]);
  Graph("readings/position_y", new_pose.position[1]);

  return {new_pose, tag_pos.pos, estimated_pose, yaw_rate};
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  if (DrivetrainOLControlTarget* ol_target =
          std::get_if<DrivetrainOLControlTarget>(&target)) {
    Graph("target/ol_velocity_x", ol_target->velocity[0]);
    Graph("target/ol_velocity_y", ol_target->velocity[1]);
    Graph("target/ol_angular_velocity", ol_target->angular_velocity);

    units::degree_t bearing = GetReadings().pose.bearing;
    units::degree_t steer_lag_compensation =
        GetPreferenceValue_unit_type<units::second_t>("steer_lag") *
        GetReadings().yaw_rate;

    Graph("target/steer_lag_compensation", steer_lag_compensation);

    auto velocity_compensated =
        ol_target->velocity.rotate(steer_lag_compensation, true);

    auto ol_calc_outputs = ol_calculator_.calculate({velocity_compensated,
        ol_target->angular_velocity, bearing,
        GetPreferenceValue_unit_type<units::feet_per_second_t>("max_speed")});

    for (int i = 0; i < 4; i++) {
      modules_[i]->SetSteerGains({GetPreferenceValue_double("steer_gains/_kP"),
          GetPreferenceValue_double("steer_gains/_kI"),
          GetPreferenceValue_double("steer_gains/_kD"),
          GetPreferenceValue_double("steer_gains/_kF")});

      SwerveModuleOLControlTarget module_target{
          .drive = ol_calc_outputs.drive_outputs[i],
          .steer = ol_calc_outputs.steer_outputs[i]};
      modules_[i]->SetTarget(module_target);
    }
  } else if (DrivetrainAccelerationControlTarget* accel_target =
                 std::get_if<DrivetrainAccelerationControlTarget>(&target)) {
    throw std::runtime_error("Acceleration control not yet implemented");
    // TODO: implement acceleration control for drivetrain
  }

  for (int i = 0; i < 4; i++)
    modules_[i]->UpdateHardware();
}

}  // namespace frc846::robot::swerve