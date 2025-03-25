#include "subsystems/abstract/gpd.h"

#include <vector>

#include "frc846/math/fieldpoints.h"
#include "frc846/wpilib/time.h"

GPDSubsystem::GPDSubsystem(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>{"GPD"},
      drivetrain_{drivetrain} {
#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    g_field.GetObject(std::to_string(i));
  }
  frc::SmartDashboard::PutData("GPDField", &g_field);
#endif

  RegisterPreference("intake_to_cam_y", 0_in);
  RegisterPreference("intake_to_cam_x", -13_in);
  RegisterPreference("cam_h_angle", 0_deg);

  RegisterPreference("max_gp_diff", 10_in);
  RegisterPreference("use_diff_thresh", 40_in);
}

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

std::pair<frc846::math::Vector2D, bool> GPDSubsystem::getBestGP(
    const std::vector<frc846::math::Vector2D> algae) {
  if (algae.size() == 0U) { return {{0_in, 0_in}, false}; }

  frc846::math::Vector2D closest_algae;

  auto robot_pose = drivetrain_->GetReadings().pose;

  if (robot_pose.velocity.magnitude() >= 2_fps) {
    units::degree_t min_angle = 180_deg;
    for (size_t i = 0; i < algae.size(); i++) {
      frc846::math::Vector2D this_algae = algae.at(i);
      units::degree_t angle = robot_pose.velocity.angleTo(this_algae, true);

      if (angle < min_angle) {
        min_angle = angle;
        closest_algae = this_algae;
      }
    }
  } else {
    units::inch_t min_dist = 1000_in;
    for (size_t i = 0; i < algae.size(); i++) {
      frc846::math::Vector2D this_algae = algae.at(i);
      units::inch_t dist = (this_algae - robot_pose.position).magnitude();

      if (dist < min_dist) {
        min_dist = dist;
        closest_algae = this_algae;
      }
    }
  }

  return {closest_algae, true};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings;
  frc846::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  g_field.SetRobotPose(frc846::math::FieldPoint::field_size_y -
                           drivetrain_readings.estimated_pose.position[1],
      drivetrain_readings.estimated_pose.position[0],
      180_deg - drivetrain_readings.estimated_pose.bearing);

  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});

  auto latency_entry = gpdTable->GetEntry("tl");
  units::second_t nt_delay =
      frc846::wpilib::CurrentFPGATime() -
      units::microsecond_t(latency_entry.GetLastChange());
  Graph("nt_delay", nt_delay);
  units::second_t latency = latency_entry.GetDouble(0.005) * 1_s + nt_delay;
  Graph("latency", latency);

  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});

  readings.gamepieces.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    readings.gamepieces.push_back(
        frc846::math::Vector2D{units::inch_t(distances[i]),
            drivetrain_readings.estimated_pose.bearing -
                drivetrain_readings.yaw_rate * latency +
                units::degree_t(theta_x[i]) +
                GetPreferenceValue_unit_type<units::degree_t>("cam_h_angle"),
            true} +
        drivetrain_readings.estimated_pose.position -
        frc846::math::Vector2D{
            drivetrain_readings.estimated_pose.velocity[0] * latency,
            drivetrain_readings.estimated_pose.velocity[1] * latency} +
        frc846::math::Vector2D{
            GetPreferenceValue_unit_type<units::inch_t>("intake_to_cam_x"),
            GetPreferenceValue_unit_type<units::inch_t>("intake_to_cam_y")}
            .rotate(drivetrain_readings.estimated_pose.bearing));
  }

  int num_gps = readings.gamepieces.size();

  // Graph("num_gps", num_gps);

  gp_spin_ += 5_deg;

#ifndef _WIN32
  for (int i = 0; i < std::min(20, num_gps); i++) {
    auto pos = readings.gamepieces[i];
    g_field.GetObject(std::to_string(i))
        ->SetPose(
            frc846::math::FieldPoint::field_size_y - pos[1], pos[0], gp_spin_);
  }
  for (int i = std::min(20, num_gps); i < 20; i++) {
    g_field.GetObject(std::to_string(i))->SetPose(100_m, 100_m, 0_deg);
  }
#endif
  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}